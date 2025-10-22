// can_driver.h
#include <iostream>

namespace CAN {
    // ==================== 常量定义 ====================
    constexpr uint16_t CAN_SEND_ID = 0x1;
    constexpr uint16_t CAN_RECEIVE_ID = 0x11;
    constexpr uint16_t CAN_BUFFER_SIZE = 128;

    // ==================== 类型定义 ====================
    using hcan_t =
    CAN_HandleTypeDef; // 前向声明，实际定义在 can.h

    struct can_fifo_buffer_t {
        uint8_t *data;
        uint16_t read_index;
        uint16_t write_index;
        uint16_t size;
    };

    // ==================== 初始化 ====================
    void can_driver_init();

    // ==================== 发送接口 ====================
    void canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);

    uint16_t can_serial_write(uint8_t *buffer, uint16_t length);

    void can_transmit();

    // ==================== 接收接口 ====================
    uint8_t canx_receive_data(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf);

    uint16_t can_serial_available();

    uint8_t can_serial_read_char();

    uint16_t can_serial_read(uint8_t *buffer, uint16_t length);

    uint16_t can_tx_buf(uint8_t *buffer, uint16_t length, can_fifo_buffer_t *tx_buffer);

    void can_para_init();


    // ==================== 回调设置 ====================
    using callback_func_t = void(*)();

    void set_can1_callback(callback_func_t func);
} // namespace CAN
