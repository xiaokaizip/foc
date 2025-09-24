// serial_logger.cpp
#include "serial_logger.h"
#include <cstring>
#include <algorithm>  // for std::min

// 构造函数
SerialLogger::SerialLogger(UART_HandleTypeDef *huart) : huart(huart) {
    // 初始化数据包尾
    std::memcpy(packet.tail, "\x00\x00\x80\x7F", 4);
}

float SerialLogger::get_velocity() {
    return command.target_velocity;
}

float SerialLogger::get_position() {
    return command.target_position;
}

float SerialLogger::get_enable() {
    return command.enabled;
}

void SerialLogger::addDataToChannel(uint8_t index, float *variablePtr) {
    if (index < 24 && variablePtr != nullptr) {
        dataPointers[index] = variablePtr;
    }
}

void SerialLogger::send_to_vofa() {
    for (int i = 0; i < 24; i++) {
        if (dataPointers[i] != nullptr) {
            packet.data[i] = *(dataPointers[i]);
        }
    }
    packet.data[23] = static_cast<float>(vofa_count++);

    HAL_UART_Transmit(huart, reinterpret_cast<uint8_t *>(&packet), sizeof(VofaPacket), HAL_MAX_DELAY);
}

// 🔁 新增：启动接收（在 main 中调用一次）
void SerialLogger::startReceive() {
    // 启动 DMA 接收 8 字节
    HAL_UART_Receive_DMA(huart, rx_buffer, 8);

}


// 🔁 新增：解包函数
void SerialLogger::unpackData() {
    // 强制对齐访问（假设小端）
    int16_t angle_x10 = *reinterpret_cast<int16_t *>(rx_buffer + 0);
    int16_t speed_x10 = *reinterpret_cast<int16_t *>(rx_buffer + 2);
    int16_t torque_x100 = *reinterpret_cast<int16_t *>(rx_buffer + 4);
    uint8_t enabled = rx_buffer[6];

    // 转换为实际物理量
    command.target_position = (float) angle_x10 / 10.0f;
    command.target_velocity = (float) speed_x10 / 10.0f;
    command.target_torque = (float) torque_x100 / 100.0f;
    command.enabled = (enabled != 0) ? 1 : 0;
}

// 🔁 新增：获取控制指令（供 FOC 控制器使用）
const ControlCommand &SerialLogger::getCommand() {
    return command;
}