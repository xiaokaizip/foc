#include "main.h"
#include <string.h>
#include "usart.h"
#include"position_sensor.h"
// 发送数据缓冲区
#pragma pack(push, 1)  // 确保1字节对齐
typedef struct {
    float data[24]; // 24个float数据
    uint8_t tail[4]; // 协议帧尾
} VofaPacket;
#pragma pack(pop)

extern EncoderData_t encoderData; // 编码器数据结构体
extern unsigned int gADC_IN10, gADC_IN11;


int vofa_count = 0;

void send_to_vofa() {
    VofaPacket packet = {.tail = {0x00, 0x00, 0x80, 0x7F}};

    packet.data[0] = encoderData.mechanical_angle;
    packet.data[1] = encoderData.electrical_angle;
    packet.data[2] = encoderData.angular_velocity;
    packet.data[3] = encoderData.rpm;
    packet.data[4] = vofa_count++;
    packet.data[5] = encoderData.cumulative_angle;
    packet.data[6] = TIM1->CCR1;;
    packet.data[7] = TIM1->CCR2;
    packet.data[8] = TIM1->CCR3;
    packet.data[9] = gADC_IN10;
    packet.data[10] = gADC_IN11;
    // 3. 通过串口发送整个数据包
    HAL_UART_Transmit(&huart2, (uint8_t *) &packet, sizeof(VofaPacket), HAL_MAX_DELAY);
}

// 示例调用
void example_usage() {
    float sensor_data[24] = {0}; // 填充实际数据

    // 模拟数据更新 (实际使用时替换为真实传感器数据)
    for (int i = 0; i < 24; i++) {
        sensor_data[i] = i * 0.1f; // 示例数据
    }

    // 发送到VOFA+
    send_to_vofa(sensor_data);
}
