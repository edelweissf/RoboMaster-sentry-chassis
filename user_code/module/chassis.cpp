#include "Chassis.h"
#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "Slam_control.h"
#include "Ui.h"
#ifdef __cplusplus // 告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

// 底盘模块 对象
Chassis chassis;

/*
  步兵底盘功率：功率优先  60 80 100
                血量优先  45 50 55
*/
// 超电模块
Super_Cap cap;

// 扭腰控制数据
fp32 swing_angle = 0.0f;
uint8_t swing_switch = 0;
uint8_t key_pressed_num_ctrl = 0;

// 小陀螺控制数据
fp32 top_angle = 0.0f;
bool_t top_switch = 0;

// 45度角对敌数据
fp32 pisa_angle = 0.0f; // 保留45度对敌前的云台相对底盘角度
bool_t pisa_switch = 0;
// 超电控制数据
extern bool_t super_cap_switch;

/**
 * @brief          slam控制模式
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         none
 */
void Chassis::chassis_slam_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    slam_move(vx_set, vy_set, wz_set);

    return;
}

void Chassis::init()
{
    chassis_RC = remote_control.get_remote_control_point();
    last_chassis_RC = remote_control.get_last_remote_control_point();

    // 设置初试状态机
    chassis_mode = CHASSIS_VECTOR_ZERO_FORCE;
    last_chassis_mode = chassis_mode;

    // 初始化底盘电机
    for (uint8_t i = 0; i < 4; ++i)
    {

        // 动力电机数据
        chassis_motive_motor[i].init(can_receive.get_chassis_motive_motor_measure_point(i));
        // 初始化pid
        fp32 motive_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        chassis_motive_motor[i].speed_pid.init(PID_SPEED, motive_speed_pid_parm, &chassis_motive_motor[i].speed, &chassis_motive_motor[i].speed_set, NULL);
        chassis_motive_motor[i].speed_pid.pid_clear();
    }

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    // 用一阶滤波代替斜波函数生成
    chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    // 初始化角度Z轴PID
    fp32 z_angle_pid_parm[5] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT};
    chassis_wz_angle_pid.init(PID_ANGLE, z_angle_pid_parm, &chassis_relative_angle, &chassis_relative_angle_set, NULL);
    chassis_wz_angle_pid.pid_clear();
    // 速度限幅设置
    x.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    x.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;

    y.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    y.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;

    z.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
    z.max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;

    // 更新一下数据
    feedback_update();
}

/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Chassis::chassis_behaviour_mode_set()
{
    // 自主运行下的模式判断
    // zxn(暂时调试用，向下时候才判断slam)
    if (slam_if_move() && switch_is_down(chassis_RC->rc.s[CHASSIS_RIGHT_CHANNEL])) // 如果slam有信号输入
    {
        chassis_mode = CHASSIS_VECTOR_SLAM;
    }
    else if (switch_is_up(chassis_RC->rc.s[CHASSIS_RIGHT_CHANNEL])) // 如果slam无底盘输入且遥控器右拨杆没上拉
    {
        chassis_mode = CHASSIS_VECTOR_SPIN;
    }
    else if (switch_is_up(chassis_RC->rc.s[CHASSIS_LEFT_CHANNEL]))
    {
        chassis_mode = CHASSIS_VECTOR_REMOTE;
    }
    else
    {
        chassis_mode = CHASSIS_VECTOR_ZERO_FORCE;
    }
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]
 * @retval         none
 */
void Chassis::set_mode()
{
    chassis_behaviour_mode_set();
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]
 * @retval         none
 */
void Chassis::feedback_update()
{

    // 更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        // 更新动力电机速度，加速度是速度的PID微分
        chassis_motive_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_motive_motor[i].motor_measure->speed_rpm;
        chassis_motive_motor[i].accel = chassis_motive_motor[i].speed_pid.data.error_delta * CHASSIS_CONTROL_FREQUENCE;
    }

    // 更新底盘x, y, z速度值,右手坐标系
    //  TODO 速度的更新可能要进行修改
    x.speed = (-chassis_motive_motor[0].speed + chassis_motive_motor[1].speed + chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    y.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed + chassis_motive_motor[2].speed + chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    z.speed = (-chassis_motive_motor[0].speed - chassis_motive_motor[1].speed - chassis_motive_motor[2].speed - chassis_motive_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    // TODO 还未完善
    // 底盘相对于云台的角度,由云台发送过来 编码器中的角度
    chassis_relative_angle = can_receive.chassis_receive.gimbal_yaw_angle;
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]       包括底盘所有信息.
 * @retval         none
 */
void Chassis::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    if (chassis_mode == CHASSIS_VECTOR_SPIN)
    {
        chassis_spin_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_mode == CHASSIS_VECTOR_SLAM)
    {
        chassis_slam_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_mode == CHASSIS_VECTOR_REMOTE)
    {
        chassis_rc_to_control_vector(vx_set, vy_set, angle_set);
    }
    else
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set);
    }
}

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Chassis::set_contorl()
{
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

    // 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);

    // “angle_set” 是旋转速度控制
    z.speed_set = angle_set;

    // 速度限幅
    x.speed_set = fp32_constrain(vx_set, x.min_speed, x.max_speed);
    y.speed_set = fp32_constrain(vy_set, y.min_speed, y.max_speed);
}

/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void Chassis::solve()
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 动力电机目标速度

    uint8_t i = 0;

    // 全向轮运动分解
    chassis_vector_to_omni_wheel_speed(wheel_speed);

    // 计算动力电机控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_motive_motor[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_motive_motor[i].speed_set *= vector_rate;
        }
    }

    // 计算pid
    for (i = 0; i < 4; i++)
    {
        // 计算动力电机的输出电流
        chassis_motive_motor[i].current_set = chassis_motive_motor[i].speed_pid.pid_calc();
    }
}

void Chassis::chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    static int spin_begin_time = 0; // 小陀螺启动时间
    static int spin_change_time = 0;
    static float spin_change = 0; // 小陀螺变速变量
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

#if IF_REMOTE_CONTROL
    // 遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set);
    chassis_control_vector(vx_set, vy_set);
#endif

    /**************************小陀螺控制输入********************************/
    if (IF_SPIN_OPEN)
    {
        spin_change = abs(sin(double(spin_change_time / SPIN_CHANGE_V))); // 将时间转化为变速变量

        if ((fabs(*vx_set) < 0.001f) && (fabs(*vy_set) < 0.001f))
        {
            if (spin_begin_time <= SPIN_BEGIN_TIME / 3)
            {
                top_angle = 5.0 / SPIN_PROPORTION * spin_change;
                spin_begin_time++;
            }
            else if (spin_begin_time <= SPIN_BEGIN_TIME / 2)
            {
                top_angle = 10.0 / SPIN_PROPORTION * spin_change;
                spin_begin_time++;
            }
            else if (spin_begin_time <= SPIN_BEGIN_TIME)
            {
                top_angle = 15.0 / SPIN_PROPORTION * spin_change;
            }
            top_angle = 15.0 / SPIN_PROPORTION * spin_change; // top_wz_ctrl;
        }
        else
            top_angle = 5.0 / SPIN_PROPORTION;
        spin_change_time++;
    }
    else
    {
        top_angle = 0;
        spin_begin_time = 0;
    }

    *vx_set = 0;
    *vy_set = 0;
    *angle_set = top_angle; // 对小陀螺的最高转速做限定
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @retval         none
 */
void Chassis::chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel, angle_channel;
    fp32 vx_set_channel, vy_set_channel, angle_set_channel;
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], angle_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    angle_set_channel = angle_channel * -CHASSIS_OPEN_RC_SCALE;

    // 一阶低通滤波代替斜波作为底盘速度输入
    chassis_cmd_slow_set_vx.first_order_filter_cali(vx_set_channel);
    chassis_cmd_slow_set_vy.first_order_filter_cali(vy_set_channel);
    chassis_cmd_slow_set_vz.first_order_filter_cali(angle_set_channel);

    // 停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_cmd_slow_set_vy.out = 0.0f;
    }
    if (angle_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_OPEN_RC_SCALE && angle_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_OPEN_RC_SCALE)
    {
        chassis_cmd_slow_set_vz.out = 0.0f;
    }

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
    *angle_set = chassis_cmd_slow_set_vz.out;
}

/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
void Chassis::chassis_vector_to_omni_wheel_speed(fp32 wheel_speed[4])
{
#if IF_SEMTRY_SOLVE
    float WHEEL_PERIMETER = 152.5;
    float CHASSIS_DECELE_RATIO = 19;
    float LENGTH_A = 293.725;
    float LENGTH_B = 293.725;
    float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14f) * CHASSIS_DECELE_RATIO * 1000;
    wheel_speed[0] = x.speed_set + y.speed_set + (LENGTH_A + LENGTH_B) * wheel_rpm_ratio * z.speed_set;  // 右前轮，左前
    wheel_speed[1] = x.speed_set - y.speed_set + (LENGTH_A + LENGTH_B) * wheel_rpm_ratio * z.speed_set;  // 右后轮，右前
    wheel_speed[2] = -x.speed_set - y.speed_set + (LENGTH_A + LENGTH_B) * wheel_rpm_ratio * z.speed_set; // 左后轮，右后
    wheel_speed[3] = -x.speed_set + y.speed_set + (LENGTH_A + LENGTH_B) * wheel_rpm_ratio * z.speed_set; // 左前轮，左后
#else
    wheel_speed[0] = -x.speed_set - y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[1] = x.speed_set - y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[2] = x.speed_set + y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
    wheel_speed[3] = -x.speed_set + y.speed_set - MOTOR_DISTANCE_TO_CENTER * z.speed_set;
#endif
}

fp32 move_top_xyz_parm[3] = {1.0, 1.0, 1.3};
fp32 chassis_power_cap_buffer = 0.0f; // 电容剩余能量

fp32 chassis_power = 0.0f;
fp32 chassis_power_limit = 0.0f;
// 缓冲能量 单位为J
fp32 chassis_power_buffer = 0.0f; // 裁判剩余缓冲能量
// fp32 chassis_power_cap_buffer = 0.0f; //电容剩余能量

/**
 * @brief          底盘功率控制
 * @param[in]
 * @retval         none
 */
void Chassis::power_ctrl()
{

    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = 0;
    referee.get_robot_id(&robot_id);

    if (robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
        can_receive.can_cmd_super_cap_power(4500);
    }
    else
    {
        referee.get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        cap.read_cap_buff(&chassis_power_cap_buffer);
        //        cap.super_number = chassis_power_cap_buffer/1400.0;
        referee.get_chassis_power_limit(&chassis_power_limit);

        if (top_switch == true && chassis_power_buffer > 10.0f)
        {
            can_receive.can_cmd_super_cap_power(uint16_t(chassis_power_limit) * 100 + 100);
        }
        else if (top_switch == false && chassis_power_buffer > 10.0f)
        {
            can_receive.can_cmd_super_cap_power(uint16_t(chassis_power_limit) * 100 + 1500);
        }
        else
        {
            can_receive.can_cmd_super_cap_power(uint16_t(chassis_power_limit) * 100);
        }
    }
    // 电流限幅
    if (super_cap_switch == true)
    {
        total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        total_current_limit = POWER_TOTAL_CURRENT_LIMIT;
    }

    total_current = 0.0f;
    // 计算原本电机电流设定
    for (uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_motive_motor[i].current_set);
    }

    if (total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        // 对动力电机进行功率控制
        chassis_motive_motor[0].current_set *= current_scale;
        chassis_motive_motor[1].current_set *= current_scale;
        chassis_motive_motor[2].current_set *= current_scale;
        chassis_motive_motor[3].current_set *= current_scale;
    }
}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void Chassis::output()
{
    // 赋值电流值
    for (int i = 0; i < 4; i++)
    {
        chassis_motive_motor[i].current_give = (int16_t)(chassis_motive_motor[i].current_set);
        if (chassis_mode == CHASSIS_VECTOR_ZERO_FORCE)
        {
            chassis_motive_motor[i].current_give = 0;
        }
    }

    // 电流输出控制,通过调整宏定义控制
    for (int i = 0; i < 4; i++)
    {
#if CHASSIS_MOTIVE_MOTOR_NO_CURRENT
        chassis_motive_motor[i].current_give = 0;
#endif
    }

    can_receive.can_cmd_chassis_motive_motor(chassis_motive_motor[0].current_give, chassis_motive_motor[1].current_give,
                                             chassis_motive_motor[2].current_give, chassis_motive_motor[3].current_give);
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @retval         返回空
 */
void Chassis::chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
