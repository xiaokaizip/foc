// serial_logger.h
#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include "main.h"  // 包含 HAL 库定义
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

// ==================== 配置 ====================
#define RX_BUFFER_SIZE        64          // DMA 接收缓冲区大小
#define MODBUS_SLAVE_ID       0xea        // Modbus 从机地址

// 寄存器映射（对应 40001 起）
#define REG_POS_OFFSET        0x65           // 目标位置 ×100
#define REG_VEL_OFFSET        0x66           // 目标速度 ×10
#define REG_TOR_OFFSET        0x67           // 目标扭矩 ×100
#define REG_CTRL_OFFSET       0x68           // 控制字
#define REG_STAT_OFFSET       0x69           // 状态字
#define MODBUS_REG_COUNT      0x70          // 总寄存器数量

// ==================== 数据结构 ====================

// Vofa+ JustFloat 协议包
typedef struct {
    float data[24];
    uint8_t tail[4]; // 0x00, 0x00, 0x80, 0x7F
} VofaPacket_t;

// 控制指令结构体
typedef struct {
    float target_position;
    float target_velocity;
    float target_torque;
    uint8_t enabled;
} ControlCommand_t;

// Modbus 串口日志器上下文（替代 C++ 类）
typedef struct {
    UART_HandleTypeDef *huart;

    // 接收缓冲区（DMA 使用）
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    // 接收到的不定长数据的长度
    uint16_t rx_index;
    //用来判断是否有更新数据
    bool is_update;

    // Modbus 寄存器池
    uint16_t modbus_registers[MODBUS_REG_COUNT];

    // 当前控制命令
    ControlCommand_t command;

    // Vofa+ 发送包
    VofaPacket_t packet;

    // Vofa 计数器
    uint32_t vofa_count;

    // 数据指针（用于 Vofa+ 发送）
    float *data_pointers[24];
} SerialLogger_t;

// ==================== 函数声明 ====================

/**
 * @brief 初始化 SerialLogger
 * @param logger 指向上下文结构体
 * @param huart UART 句柄
 */
void SerialLogger_Init(SerialLogger_t *logger, UART_HandleTypeDef *huart);

/**
 * @brief 启动 DMA 接收和空闲中断
 */
void SerialLogger_StartReceive(SerialLogger_t *logger);

/**
 * @brief 空闲中断回调（在 UART IRQHandler 中调用）
 */
void SerialLogger_OnReceiveComplete(SerialLogger_t *logger);

/**
 * @brief 获取当前控制命令（只读）
 */
const ControlCommand_t *SerialLogger_GetCommand(SerialLogger_t *logger);

/**
 * @brief 添加变量到 Vofa+ 通道
 */
void SerialLogger_AddDataToChannel(SerialLogger_t *logger, uint8_t index, float *variable_ptr);

/**
 * @brief 发送数据到 Vofa+
 */
void SerialLogger_SendToVofa(SerialLogger_t *logger);

// ==================== 内部函数（可选导出） ====================
uint16_t SerialLogger_CRC16(uint8_t *buf, uint16_t len);

void SerialLogger_ProcessModbusFrame(SerialLogger_t *logger, uint8_t *frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_LOGGER_H
