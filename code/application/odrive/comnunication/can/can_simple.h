//
// Created by SXF-Admin on 25-10-22.
//

#ifndef CAN_SIMPLE_H
#define CAN_SIMPLE_H
#include <cstdint>

#include "FastMath.h"

// ==================== 常量定义 ====================
constexpr uint16_t CAN_SEND_ID = 0x11;
constexpr uint16_t CAN_RECEIVE_ID = 0x1;
constexpr uint16_t CAN_BUFFER_SIZE = 128;


class CANSimple {
public:
    void do_command(unsigned short id, unsigned char *data);

    std::array<unsigned char, 8> packet_data(); // 返回 std::array
};

#endif //CAN_SIMPLE_H
