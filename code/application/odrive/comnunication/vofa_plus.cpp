#include "main.h"
#include <string.h>
#include "usart.h"
#include "PositionSensor/PositionSensor.h"
#include "foc.h"
#include "serial_common.h"
// 发送数据缓冲区
#pragma pack(push, 1)  // 确保1字节对齐
typedef struct {
    float data[24]; // 24个float数据
    uint8_t tail[4]; // 协议帧尾
} VofaPacket;
#pragma pack(pop)

extern unsigned int gADC_IN10, gADC_IN11;
extern ControllerStruct controller;
extern PositionSensorAM5047 ps;

int vofa_count = 0;

void send_to_vofa() {
    VofaPacket packet = {.tail = {0x00, 0x00, 0x80, 0x7F}};

    packet.data[0] = ps.GetElecPosition();
    packet.data[1] = ps.GetElecVelocity();
    packet.data[2] = ps.GetMechPosition();
    packet.data[3] = ps.GetMechVelocity();
    packet.data[4] = controller.theta_elec;
    packet.data[5] = controller.adc2_offset;
    packet.data[6] = TIM1->CCR1;
    packet.data[7] = TIM1->CCR2;
    packet.data[8] = TIM1->CCR3;
    packet.data[9] = controller.i_a;
    packet.data[10] = controller.i_b;
    packet.data[11] = controller.i_c;

    packet.data[12] = controller.dtc_u;
    packet.data[13] = controller.dtc_v;
    packet.data[14] = controller.dtc_w;
    packet.data[15] = controller.kd;
    packet.data[16] = controller.kp;
    packet.data[17] = controller.p_des;
    packet.data[18] = controller.v_des;
    packet.data[19] = controller.i_d_ref;
    packet.data[20] = controller.i_q_ref;
    packet.data[21] = controller.v_d;
    packet.data[22] = controller.i_q;
    packet.data[23] = state;
    // packet.data[19] = controller.i_c;
    // packet.data[6] = E_OFFSET;

    // 3. 通过串口发送整个数据包
    HAL_UART_Transmit(&huart4, (uint8_t *) &packet, sizeof(VofaPacket), HAL_MAX_DELAY);
}

// 示例调用
void example_usage() {
    float sensor_data[24] = {0}; // 填充实际数据

    // 模拟数据更新 (实际使用时替换为真实传感器数据)
    for (int i = 0; i < 24; i++) {
        sensor_data[i] = i * 0.1f; // 示例数据
    }

    // 发送到VOFA+
}
