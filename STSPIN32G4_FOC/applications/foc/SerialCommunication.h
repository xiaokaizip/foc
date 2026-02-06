//
// Created by SXF-Admin on 26-1-27.
//

#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <cstdint>
#include <cstdarg>
#include "RingBuffer.h"

#define FRAME_HEADER        0xA5
#define MAX_FLOATS_PER_FRAME 64
#define MAX_PAYLOAD_SIZE    (MAX_FLOATS_PER_FRAME * sizeof(float))
#define CRC16_POLY          0x1021


struct FrameConfig {
    bool use_crc;
    uint8_t sync1; // 通常 0xA5
    uint8_t sync2; // 通常 0x5A
    uint8_t packet_type; // 用户自定义包类型（如 0x01, 0x02...）
    bool timestamp_as_float; // false → uint32_t, true → float
};

inline constexpr FrameConfig FULL_PROTOCOL = {
    .use_crc = true,
    .sync1 = 0xA5,
    .sync2 = 0x5A,
    .packet_type = 0x01, // 默认包类型，可被覆盖
    .timestamp_as_float = false
};

inline constexpr FrameConfig SIMPLE_PROTOCOL = {
    .use_crc = false,
    .sync1 = 0x00,
    .sync2 = 0x00,
    .packet_type = 0x00, // unused in simple mode
    .timestamp_as_float = true
};

////////////////////////////////接收数据////////////////////////////////
constexpr uint8_t CMD_REF_PARAMS = 0x10;

// 全局接收状态
enum RxState {
    WAIT_SYNC1,
    WAIT_SYNC2,
    WAIT_PACKET_TYPE,
    WAIT_COUNT,
    WAIT_TIMESTAMP,
    WAIT_DATA,
    WAIT_CRC_LO,
    WAIT_CRC_HI
};

struct RxFrame {
    uint8_t packet_type;
    uint8_t count;
    uint32_t timestamp;
    float data[3]; // 最多3个float
    uint16_t crc_received;
    size_t bytes_received;
    uint8_t raw_buffer[32]; // 足够存一帧
};

typedef struct packet {
    float refSpeed;
    float refPosition;
    float refCurrent;
} refParam_t;


class SerialCommunication {
public:
    static void Init();

    static uint16_t crc16(const uint8_t *data, size_t len);

    static size_t packFrame(uint8_t *out,
                            uint32_t timestamp,
                            const float *data,
                            size_t count,
                            const FrameConfig &config);

    // 主日志接口：允许指定协议
    static void logData(const float *data, size_t count, const FrameConfig &config = FULL_PROTOCOL);


    // 也可显式指定协议
    template<typename... Args>
    static void printFloatsWithConfig(const FrameConfig &config, Args... args);

    // 在 SerialCommunication 类中添加：
    static void logDataWithPacketType(uint8_t packet_type, const float *data, size_t count);

    template<typename... Args>
    static void printFloatsWithPacketType(uint8_t packet_type, Args... args);

    static bool parseRefParamFrame(); // 从 rxQueue 尝试解析一帧
    static refParam_t refParam;
};

// 全局函数声明（实现在 .cpp 中）
void initProtocol(); // 发送时间单位（1ms）
void sendDataPeriodically(); // 定时触发发送
void startTransmission(); // 启动 DMA 发送

// ========== 模板实现（必须在头文件中） ==========

void Receive_IRQHandler(void);

template<typename... Args>
inline void SerialCommunication::printFloatsWithConfig(const FrameConfig &config, Args... args) {
    constexpr size_t num_floats = sizeof...(args);
    static_assert(num_floats <= MAX_FLOATS_PER_FRAME,
                  "Too many arguments for printFloats! Exceeds MAX_FLOATS_PER_FRAME.");

    float floatArray[num_floats] = {static_cast<float>(args)...};
    logData(floatArray, num_floats, config);
}

inline void SerialCommunication::logDataWithPacketType(uint8_t packet_type, const float *data, size_t count) {
    FrameConfig cfg = FULL_PROTOCOL;
    cfg.packet_type = packet_type;
    logData(data, count, cfg);
}

template<typename... Args>
inline void SerialCommunication::printFloatsWithPacketType(uint8_t packet_type, Args... args) {
    constexpr size_t num = sizeof...(args);
    static_assert(num <= MAX_FLOATS_PER_FRAME, "Too many floats!");
    float arr[num] = {static_cast<float>(args)...};
    logDataWithPacketType(packet_type, arr, num);
}


#endif // SERIAL_COMMUNICATION_H
