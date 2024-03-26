#ifndef __SLAM_CONTROL
#define __SLAM_CONTROL

#include "main.h"
#include "struct_typedef.h"

#include "bsp_usart.h"

#define SLAM_BUFFER_LEN 200
#define NOW 0
#define LAST 1

#define SLAM_LEN_HEADER 1      // 帧头长
#define SLAM_LEN_DATA 9        // 数据段长度,可自定义
#define SLAM_SEND_LEN_PACKED 0 // 暂时没有发送，备用
#define SLAM_READ_LEN_PACKED 11 // 接受数据包长度

#define SLAM_BEGIN (0xA8) // 可更改
#define SLAM_END (0xFF)   // 帧尾

// STM32接收,直接将串口接收到的数据拷贝进结构体 18帧
typedef __packed struct // 11 Byte
{
    /* 头 */
    uint8_t BEGIN; // 帧头起始位,暂定0xA8

    /* 数据 */
    uint8_t isMoving; // 是否移动  0不移动  1移动
    fp32 x_speed;     // x方向的速度
    fp32 y_speed;     // y 方向的速度

    uint8_t END;

} SlamRecvData_t;

extern uint8_t CmdID;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t Slam_Buffer[2][SLAM_BUFFER_LEN]; // Slam数据暂存

// 命令码ID,用来判断接收的是什么数据
void slam_read_data(uint8_t *ReadFormUart1);

void slam_move(float *vx, float *vy, float *vz);

extern void slam_init();

bool_t slam_if_move();

#endif
