// serial_logger.c
#include "serial_logger.h"
#include <string.h>
#include <stdio.h>

// 初始化函数
void SerialLogger_Init(SerialLogger_t *logger, UART_HandleTypeDef *huart) {
    if (!logger || !huart) return;

    // 初始化成员
    logger->huart = huart;
    memset(logger->rx_buffer, 0, RX_BUFFER_SIZE);
    memset(logger->modbus_registers, 0, sizeof(logger->modbus_registers));
    memset(logger->data_pointers, 0, sizeof(logger->data_pointers));

    // 初始化 Vofa 包尾
    logger->packet.tail[0] = 0x00;
    logger->packet.tail[1] = 0x00;
    logger->packet.tail[2] = 0x80;
    logger->packet.tail[3] = 0x7F;

    logger->vofa_count = 0;

    // 初始化命令
    logger->command.target_position = 0.0f;
    logger->command.target_velocity = 0.0f;
    logger->command.target_torque = 0.0f;
    logger->command.enabled = 0;
}

// 启动接收（DMA + IDLE 中断）
void SerialLogger_StartReceive(SerialLogger_t *logger) {
    if (!logger || !logger->huart) return;


    // 启动 DMA 接收（循环模式）
    HAL_UART_Receive_DMA(logger->huart, logger->rx_buffer, RX_BUFFER_SIZE);

    // 开启空闲中断
    __HAL_UART_ENABLE_IT(logger->huart, UART_IT_IDLE);
    __HAL_DMA_DISABLE_IT(logger->huart->hdmarx, DMA_IT_TC);
}

// 空闲中断回调（在 UART_IRQHandler 中调用）
void SerialLogger_OnReceiveComplete(SerialLogger_t *logger) {
    if (!logger || !logger->huart) return;

    // 获取 DMA 剩余计数
    uint32_t remain = __HAL_DMA_GET_COUNTER(logger->huart->hdmarx);
    uint32_t received_len = (RX_BUFFER_SIZE - remain - logger->rx_index) % RX_BUFFER_SIZE;
    if (received_len > 0) {
        // 处理 Modbus 帧
        SerialLogger_ProcessModbusFrame(logger, logger->rx_buffer, received_len);
    }
    logger->rx_index = (logger->rx_index + received_len) % RX_BUFFER_SIZE;
}

// CRC16 计算
uint16_t SerialLogger_CRC16(uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 处理 Modbus 帧
void SerialLogger_ProcessModbusFrame(SerialLogger_t *logger, uint8_t *frame, uint16_t len) {
    if (len < 3) return;

    unsigned char buffer[RX_BUFFER_SIZE];
    for (int i = 0; i < len; i++) {
        buffer[i] = frame[(logger->rx_index + i) % RX_BUFFER_SIZE];
    }
    uint8_t slave_id = buffer[0];
    uint8_t func_code = buffer[1];

    // 检查从机地址
    if (slave_id != MODBUS_SLAVE_ID && slave_id != 0x00) return;

    // CRC 校验
    uint16_t crc_recv = (buffer[len - 1] << 8) | buffer[len - 2];
    uint16_t crc_calc = SerialLogger_CRC16(buffer, len - 2);
    if (crc_recv != crc_calc) return;

    uint8_t response[16] = {0};
    uint8_t resp_len = 0;

    switch (func_code) {
        case 0x03: {
            // 读保持寄存器
            uint16_t start_addr = (buffer[2] << 8) | buffer[3];
            uint16_t reg_count = (buffer[4] << 8) | buffer[5];

            if (start_addr >= MODBUS_REG_COUNT || reg_count == 0 || reg_count > 10) break;

            response[0] = MODBUS_SLAVE_ID;
            response[1] = 0x03;
            response[2] = reg_count * 2;
            resp_len = 3;

            for (int i = 0; i < reg_count; i++) {
                uint16_t val = logger->modbus_registers[start_addr + i];
                response[resp_len++] = val >> 8;
                response[resp_len++] = val & 0xFF;
            }
            break;
        }

        case 0x06: {
            // 写单寄存器
            uint16_t addr = (buffer[2] << 8) | buffer[3];
            uint16_t value = (buffer[4] << 8) | buffer[5];

            if (addr < MODBUS_REG_COUNT) {
                logger->modbus_registers[addr] = value;

                // 更新控制命令
                if (addr == REG_POS_OFFSET) {
                    logger->command.target_position = (float) value / 100.0f;
                } else if (addr == REG_VEL_OFFSET) {
                    logger->command.target_velocity = (float) value / 10.0f;
                } else if (addr == REG_TOR_OFFSET) {
                    logger->command.target_torque = (float) value / 100.0f;
                } else if (addr == REG_CTRL_OFFSET) {
                    logger->command.enabled = (value & 0x01) ? 1 : 0;
                }

                // 回显
                response[0] = MODBUS_SLAVE_ID;
                response[1] = 0x06;
                response[2] = buffer[2];
                response[3] = buffer[3];
                response[4] = buffer[4];
                response[5] = buffer[5];
                resp_len = 6;
            }
            break;
        }

        case 0x10: {
            // 写多个寄存器
            uint16_t start_addr = (buffer[2] << 8) | buffer[3];
            uint16_t reg_count = (buffer[4] << 8) | buffer[5];
            uint8_t byte_count = buffer[6];

            if (byte_count != reg_count * 2 || start_addr + reg_count > MODBUS_REG_COUNT) break;

            for (int i = 0; i < reg_count; i++) {
                uint16_t val = (buffer[7 + i * 2] << 8) | buffer[8 + i * 2];
                logger->modbus_registers[start_addr + i] = val;

                if (start_addr + i == REG_POS_OFFSET) {
                    logger->command.target_position = (float) val / 100.0f;
                } else if (start_addr + i == REG_VEL_OFFSET) {
                    logger->command.target_velocity = (float) val / 10.0f;
                } else if (start_addr + i == REG_TOR_OFFSET) {
                    logger->command.target_torque = (float) val / 100.0f;
                } else if (start_addr + i == REG_CTRL_OFFSET) {
                    logger->command.enabled = (val & 0x01) ? 1 : 0;
                }
            }

            response[0] = MODBUS_SLAVE_ID;
            response[1] = 0x10;
            response[2] = buffer[2];
            response[3] = buffer[3];
            response[4] = buffer[4];
            response[5] = buffer[5];
            resp_len = 6;
            break;
        }

        default:
            return;
    }

    if (resp_len > 0) {
        uint16_t crc = SerialLogger_CRC16(response, resp_len);
        response[resp_len++] = crc & 0xFF;
        response[resp_len++] = crc >> 8;
        HAL_UART_Transmit(logger->huart, response, resp_len, HAL_MAX_DELAY);
        logger->is_update = true;
    }
    // ✅ 5. 关键：立即重新启动 DMA 接收！

    HAL_UART_Receive_DMA(logger->huart, logger->rx_buffer, RX_BUFFER_SIZE);

    // ✅ 6. 再次开启 IDLE 中断（确保下次还能触发）
    __HAL_UART_ENABLE_IT(logger->huart, UART_IT_IDLE);
}

// 获取控制命令
const ControlCommand_t *SerialLogger_GetCommand(SerialLogger_t *logger) {
    return &logger->command;
}

// 添加数据到 Vofa+ 通道
void SerialLogger_AddDataToChannel(SerialLogger_t *logger, uint8_t index, float *variable_ptr) {
    if (index < 24 && variable_ptr != NULL) {
        logger->data_pointers[index] = variable_ptr;
    }
}

// 发送数据到 Vofa+
void SerialLogger_SendToVofa(SerialLogger_t *logger) {
    for (int i = 0; i < 24; i++) {
        if (logger->data_pointers[i] != NULL) {
            logger->packet.data[i] = *(logger->data_pointers[i]);
        }
    }
    logger->packet.data[23] = (float) (logger->vofa_count++);

    HAL_UART_Transmit(logger->huart, (uint8_t *) &logger->packet, sizeof(VofaPacket_t), HAL_MAX_DELAY);
}


