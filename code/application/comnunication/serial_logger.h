// serial_logger.h
#ifndef FOC_SERIAL_LOGGER_H
#define FOC_SERIAL_LOGGER_H

#include "main.h"
#include "position_sensor.h"

// Vofa+ JustFloat 协议包
typedef struct {
    float data[24];
    uint8_t tail[4];  // 0x00, 0x00, 0x80, 0x7F
} VofaPacket;

// 控制指令结构体（外部可访问）
typedef struct {
    float target_position;   // 单位：rad
    float target_velocity;   // 单位：根据你系统定义（如 rad/s）
    float target_torque;  // 单位：N·m
    uint8_t enabled;      // 0: disable, 1: enable
} ControlCommand;

class SerialLogger {
public:
    SerialLogger(UART_HandleTypeDef *huart);

    float get_velocity();

    float get_position();

    float get_enable();

    void addDataToChannel(uint8_t index, float *variablePtr);

    void send_to_vofa();

    // 🔁 新增：启动接收
    void startReceive();

    // 🔁 新增：获取最新控制指令
    const ControlCommand &getCommand();

    // 💡 回调函数（可由中断调用）
    void onReceiveComplete();

    void unpackData();

private:
    UART_HandleTypeDef *huart;
    float velocity = 0;
    float position = 0;

    float *dataPointers[24] = {nullptr};
    VofaPacket packet = {.tail = {0x00, 0x00, 0x80, 0x7F}};
    uint32_t vofa_count = 0;

    // 📥 接收缓冲区（DMA用）
    uint8_t rx_buffer[8];  // 固定8字节

    // 📦 解包后的控制命令
    ControlCommand command = {0};

    // 🧠 解包函数

};

#endif // FOC_SERIAL_LOGGER_H