// can_driver.cpp
/**
 * @file    can_driver.cpp
 * @brief   单CAN通道驱动 C++ 实现（仅支持 hcan1）
 * @author  AI Assistant
 * @date    2025-10-22
 */

extern "C" {
#include "main.h"
#include "can.h"
#include "string.h"
}

#include <cstdint>
#include <array>

#include "can_common.h"
#include "can_simple.h"

CANSimple CANSimple_;

namespace CAN {
    // ==================== 全局变量 ====================
    uint8_t can_buff[CAN_BUFFER_SIZE] = {0};
    can_fifo_buffer_t can_rx_buffer;
    uint8_t can_rx_buff[CAN_BUFFER_SIZE];

    can_fifo_buffer_t can_tx_buffer;
    uint8_t can_tx_buff[CAN_BUFFER_SIZE];

    // 发送临时缓冲
    uint16_t tx_len1 = 0;
    uint8_t tx_msg[8];

    // 回调函数指针
    callback_func_t can1_callback_ptr = nullptr;

    // ==================== 内部函数声明 ====================
    void can_filter_init();

    void can_bsp_init();

    uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

    uint8_t canx_bsp_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf);

    void can1_rx_callback();

    // ==================== BSP 层实现 ====================

    void can_bsp_init() {
        can_filter_init();
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    void can_filter_init() {
        CAN_FilterTypeDef can_filter_st{};
        can_filter_st.FilterActivation = ENABLE;
        can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
        can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
        can_filter_st.FilterIdHigh = 0x0000;
        can_filter_st.FilterIdLow = 0x0000;
        can_filter_st.FilterMaskIdHigh = 0x0000;
        can_filter_st.FilterMaskIdLow = 0x0000;
        can_filter_st.FilterBank = 0;
        can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
        //
        // can_filter_st.SlaveStartFilterBank = 14;
        // can_filter_st.FilterBank = 14;
        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len) {
        CAN_TxHeaderTypeDef tx_header{};
        tx_header.StdId = id;
        tx_header.ExtId = 0;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = len;

        uint32_t mailbox;

        if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox) != HAL_OK) {
            if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox) != HAL_OK) {
                HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
            }
        }
        return 0;
    }

    uint8_t canx_bsp_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf) {
        CAN_RxHeaderTypeDef rx_header{};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK) {
            *rec_id = static_cast<uint16_t>(rx_header.StdId);
            return rx_header.DLC;
        }
        return 0;
    }

    // HAL 中断回调（C 函数）
    extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
        if (hcan == &hcan1) {
            CAN::can1_rx_callback();
        }
    }

    void can1_rx_callback() {
        if (can1_callback_ptr) {
            can1_callback_ptr();
        } else {
            // 默认行为：接收匹配 ID 的数据到缓冲区
            uint8_t data[8];
            uint16_t rec_id;
            uint8_t len = canx_bsp_receive(&hcan1, &rec_id, data);
            //在这里接收数据，做数据解析。
            CANSimple_.do_command(rec_id, data);
            if (len > 0 && rec_id == CAN_RECEIVE_ID) {
                std::array<uint8_t, 8> tx_data = CANSimple_.packet_data();
                canx_bsp_send_data(&hcan1, CAN_SEND_ID, tx_data.data(), tx_data.size());
            }
        }
    }

    // ==================== 驱动层实现 ====================

    void can_driver_init() {
        can_para_init();
        can_bsp_init();
    }

    void canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len) {
        canx_bsp_send_data(hcan, id, data, len);
    }

    uint8_t canx_receive_data(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf) {
        return canx_bsp_receive(hcan, rec_id, buf);
    }

    void can_para_init() {
        can_tx_buffer.data = can_tx_buff;
        can_tx_buffer.size = CAN_BUFFER_SIZE;
        can_tx_buffer.read_index = 0;
        can_tx_buffer.write_index = 0;

        can_rx_buffer.data = can_rx_buff;
        can_rx_buffer.size = CAN_BUFFER_SIZE;
        can_rx_buffer.read_index = 0;
        can_rx_buffer.write_index = 0;
    }

    uint8_t can_serial_read_char() {
        uint8_t ch = can_rx_buffer.data[can_rx_buffer.read_index];
        can_rx_buffer.read_index = (can_rx_buffer.read_index + 1) % can_rx_buffer.size;
        return ch;
    }

    uint16_t can_serial_read(uint8_t *buffer, uint16_t length) {
        for (uint16_t i = 0; i < length; ++i) {
            buffer[i] = can_rx_buffer.data[can_rx_buffer.read_index];
            can_rx_buffer.read_index = (can_rx_buffer.read_index + 1) % can_rx_buffer.size;
        }
        return length;
    }

    uint16_t can_serial_available() {
        uint16_t r = can_rx_buffer.read_index;
        uint16_t w = can_rx_buffer.write_index;
        if (r > w) {
            return CAN_BUFFER_SIZE + w - r;
        } else if (r < w) {
            return w - r;
        }
        return 0;
    }

    uint16_t can_tx_buf(uint8_t *buffer, uint16_t length, can_fifo_buffer_t *tx_buffer) {
        for (uint16_t i = 0; i < length; ++i) {
            tx_buffer->data[tx_buffer->write_index] = buffer[i];
            tx_buffer->write_index = (tx_buffer->write_index + 1) % tx_buffer->size;
        }
        return length;
    }

    uint16_t can_serial_write(uint8_t *buffer, uint16_t length) {
        return can_tx_buf(buffer, length, &can_tx_buffer);
    }

    uint16_t can_tx_available(can_fifo_buffer_t *tx_buffer) {
        uint16_t r = tx_buffer->read_index;
        uint16_t w = tx_buffer->write_index;
        if (r > w) {
            return tx_buffer->size + w - r;
        } else if (r < w) {
            return w - r;
        }
        return 0;
    }

    void can_transmit() {
        tx_len1 = can_tx_available(&can_tx_buffer);
        if (tx_len1 == 0 || CAN_SEND_ID == 0) return;

        uint16_t temp;
        if (tx_len1 > 8) {
            temp = CAN_BUFFER_SIZE - can_tx_buffer.read_index;
            if (temp < 8) {
                memcpy(tx_msg, &can_tx_buffer.data[can_tx_buffer.read_index], temp);
                memcpy(&tx_msg[temp], &can_tx_buffer.data[0], 8 - temp);
            } else {
                memcpy(tx_msg, &can_tx_buffer.data[can_tx_buffer.read_index], 8);
            }
            canx_send_data(&hcan1, CAN_SEND_ID, tx_msg, 8);
            can_tx_buffer.read_index = (can_tx_buffer.read_index + 8) % CAN_BUFFER_SIZE;
        } else {
            temp = CAN_BUFFER_SIZE - can_tx_buffer.read_index;
            if (temp < tx_len1) {
                memcpy(tx_msg, &can_tx_buffer.data[can_tx_buffer.read_index], temp);
                memcpy(&tx_msg[temp], &can_tx_buffer.data[0], tx_len1 - temp);
            } else {
                memcpy(tx_msg, &can_tx_buffer.data[can_tx_buffer.read_index], tx_len1);
            }
            canx_send_data(&hcan1, CAN_SEND_ID, tx_msg, tx_len1);
            can_tx_buffer.read_index = (can_tx_buffer.read_index + tx_len1) % CAN_BUFFER_SIZE;
        }
    }
} // namespace CAN
