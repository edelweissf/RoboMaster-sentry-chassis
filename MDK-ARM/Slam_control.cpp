#include "Slam_control.h"
#include "Remote_control.h"
#include "struct_typedef.h"
#include "string.h"
#include "tim.h"
#include "CRC8_CRC16.h"
#include "Chassis.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t Slam_Buffer[2][SLAM_BUFFER_LEN]; // Slam数据暂存
extern Chassis chassis;

// 是否需要移动
bool_t if_move = FALSE;

SlamRecvData_t SlamRecvData; // 接收数据结构体

void slam_read_data(uint8_t *ReadFormUart1);

void slam_move(float *vx, float *vy);

void slam_init()
{
    usart1_init(Slam_Buffer[0], Slam_Buffer[1], SLAM_BUFFER_LEN);

    SlamRecvData.isMoving = 0; // 是否移动  0不移动  1移动
    SlamRecvData.x_speed = 0;  // x方向的速度
    SlamRecvData.y_speed = 0;  // y 方向的速度
}

uint8_t Slam_Time_Test[2] = {0}; // 当前数据和上一次数据
uint8_t Slam_Ping = 0;           // 发送时间间隔

/**
 * @brief  读取Slam信息
 * @param  uart1缓存数据
 * @retval void
 * @attention  IRQ执行
 */
void slam_read_data(uint8_t *ReadFormUart)
{

    // 判断帧头数据是否为0xA5
    if (ReadFormUart[0] == SLAM_BEGIN)
    {
        // 判断帧头数据是否为0xff
        if (ReadFormUart[8] == SLAM_END)
        {

            // 接收数据拷贝
            memcpy(&SlamRecvData, ReadFormUart, SLAM_READ_LEN_PACKED);

            if (SlamRecvData.isMoving == 1)
                if_move = TRUE; // 移动
            else
                if_move = FALSE; // 不移动

            // //帧计算
            // Vision_Time_Test[NOW] = xTaskGetTickCount();
            // Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
            // Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
        }
    }
}

void slam_move(float *vx, float *vy)
{
    *vx = SlamRecvData.x_speed;
    *vy = SlamRecvData.y_speed;
}

bool_t slam_if_move()
{
    return if_move;
}
