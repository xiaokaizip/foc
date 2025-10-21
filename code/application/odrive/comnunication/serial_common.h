// serial_comm.h
#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "main.h"  // HAL 库头文件
#include "usart.h" // UART handle
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>

// 假设你的状态枚举如下（根据你的实际定义调整）
enum State {
    REST_MODE,
    CALIBRATION_MODE,
    MOTOR_MODE,
    SETUP_MODE,
    ENCODER_MODE,
    OPEN_LOOP_MODE,
    VELOCITY_MODE,
    POSITION_MODE,
};

extern int state;
extern int state_change;


// 串口通信初始化
void serial_init();

// 主循环中调用的处理函数
void serial_process();


#endif // SERIAL_COMM_H
