//
// Created by SXF-Admin on 26-1-27.
//


/*
 *1.读取单片机的运行时间，在每一个数据帧中增加读取到时时间戳。
 *2.初始化配置时，将单片最小的时间单位发送给上位机，用以同步时间。
 *3.使用类似于print的写法，写入任意长度的浮点数数据，然后再将数据打包，传入帧头的参数，进行CRC校验。
 *4.将打包好的数据写入队列中，队列要足够大。
 *5.然后固定周期或是队列满了之后进行数据传输，发送给上位机。在传输数据的时候，使用队列2来接收数据
 *6，使用cpp编写
 **/

#include "SerialCommunication.h"
#include "RingBuffer.h"
#include "usart.h"
#include <cstring>
#include <cstdio>
#include <string>
#include "lwprintf/lwprintf.h"
#include <vector>
// 队列大小：例如 4KB
#define TX_QUEUE_SIZE 512
#define RX_QUEUE_SIZE 64
#define BUFFER_COUNT 2

static uint8_t dmaBuffers[BUFFER_COUNT][TX_QUEUE_SIZE];
static uint8_t rxDmaBuffer[RX_QUEUE_SIZE];

static volatile uint8_t currentBuffer = 0; // 当前正在被 DMA 使用的 buffer 索引
static volatile bool dmaActive = false; // 是否有 DMA 传输正在进行


RingBuffer txQueue(TX_QUEUE_SIZE);
RingBuffer rxQueue(RX_QUEUE_SIZE);

// CRC16-CCITT
uint16_t SerialCommunication::crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (data[i] << 8);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

size_t SerialCommunication::packFrame(
    uint8_t *out,
    uint32_t timestamp,
    const float *data,
    size_t count,
    const FrameConfig &config
) {
    if (count > MAX_FLOATS_PER_FRAME) count = MAX_FLOATS_PER_FRAME;
    // 安全：count 必须 ≤ 255（uint8_t 范围）
    if (count > 255) count = 255;

    size_t payloadSize = count * sizeof(float);
    size_t offset = 0;

    if (config.use_crc) {
        // FULL PROTOCOL: A5 5A + packet_type + count + timestamp + data + CRC
        out[offset++] = config.sync1; // 0xA5
        out[offset++] = config.sync2; // 0x5A
        out[offset++] = config.packet_type; // 用户自定义类型
        out[offset++] = static_cast<uint8_t>(count); // ← 新增：数据数量
    } else {
        // SIMPLE PROTOCOL: 固定魔数头（无 count，无 crc）
        out[offset++] = 0x00;
        out[offset++] = 0x00;
        out[offset++] = 0x80;
        out[offset++] = 0x7F;
        float count_float = static_cast<float>(count);
        memcpy(out + offset, &count_float, sizeof(count_float));
        offset += 4;
    }

    // 时间戳
    if (config.timestamp_as_float) {
        float ts_f = static_cast<float>(timestamp);
        memcpy(out + offset, &ts_f, sizeof(ts_f));
    } else {
        memcpy(out + offset, &timestamp, sizeof(timestamp)); // uint32_t
    }
    offset += 4;

    // 浮点数据
    memcpy(out + offset, data, payloadSize);
    offset += payloadSize;

    // CRC（仅 full protocol）
    if (config.use_crc) {
        uint16_t crc = crc16(out, offset); // 校验从 0xA5 开始到 data 结束
        memcpy(out + offset, &crc, sizeof(crc));
        offset += 2;
    }

    return offset;
}


void SerialCommunication::logData(const float *data, size_t count, const FrameConfig &config) {
    uint8_t frame[6 + MAX_PAYLOAD_SIZE + 2]; // 足够大
    uint32_t ts = HAL_GetTick();
    size_t len = packFrame(frame, ts, data, count, config);

    if (!txQueue.push(frame, len)) {
        // 队列满处理
    }
}

// 初始化：向上位机发送时间单位（1ms）
void initProtocol() {
    uint8_t initMsg[] = {0xAA, 0x55, 0x01, 0x00}; // 示例：表示最小单位为 1ms
    HAL_UART_Transmit(&huart1, initMsg, sizeof(initMsg), HAL_MAX_DELAY);
}

// 每 10ms 调用一次（可在 SysTick_Handler 中调用）
void sendDataPeriodically() {
    static bool transmitting = false;
    if (transmitting) return;

    if (txQueue.available() > 0 || txQueue.freeSpace() == 0) {
        startTransmission();
    }
}


void startTransmission() {
    if (dmaActive) return; // 已有传输在进行

    // 选择下一个 buffer（非当前 DMA 使用的）
    uint8_t nextBuf = currentBuffer ^ 1; // 切换 0<->1

    // 从队列填充到 nextBuf
    size_t len = txQueue.pop(dmaBuffers[nextBuf], TX_QUEUE_SIZE);
    if (len == 0) return;

    // 启动 DMA 发送 nextBuf
    currentBuffer = nextBuf; // 标记这个 buffer 正在被使用
    dmaActive = true;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, dmaBuffers[nextBuf], len);
    if (status != HAL_OK) {
        dmaActive = false; // 失败，释放
        // 可选：将数据重新 push 回队列？或丢弃
    }
}

// 在 usart.c 或 main.cpp 中（C++ 需 extern "C"）
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        dmaActive = false; // 本次传输完成
    }
}


void SerialCommunication::Init() {
    // 启动 DMA 循环接收（注意：不是 Circular 模式！而是普通模式 + 重载）
    // 但实际上我们用 IDLE，所以只需启动一次大 buffer
    HAL_UART_Receive_DMA(&huart1, rxDmaBuffer, RX_QUEUE_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

// SerialCommunication.cpp 全局
static RxState rxState = WAIT_SYNC1;
static RxFrame currentFrame;
static size_t dataBytesReceived = 0;
static size_t timestampBytesReceived = 0;

extern "C" void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        // 清除 IDLE 标志（读 SR 再读 DR）
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // 获取 DMA 已接收的数据量
        uint32_t dma_counter = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        uint32_t received_bytes = RX_QUEUE_SIZE - dma_counter;

        // 将数据推入 RingBuffer
        if (received_bytes > 0 && received_bytes <= RX_QUEUE_SIZE) {
            rxQueue.push(rxDmaBuffer, received_bytes);
        }

        // 重新启动 DMA 接收（关键！）
        HAL_UART_Receive_DMA(&huart1, rxDmaBuffer, RX_QUEUE_SIZE);
    }
    HAL_UART_IRQHandler(&huart1);
    // 其他中断（如错误）可自行处理
}


//接收的指令，现在为了调试方便还是使用之前传输字符串的方式
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    }
}

refParam_t SerialCommunication::refParam = {0.0f, 0.0f, 0.0f};

bool SerialCommunication::parseRefParamFrame() {
    // 至少需要：2(sync)+1(type)+1(count)+4(ts)+12(data)+2(crc) = 22 字节
    if (rxQueue.available() < 22) return false;

    // 偷看前几个字节（不 pop）
    uint8_t peekBuf[8];
    size_t peeked = rxQueue.peek(peekBuf, sizeof(peekBuf));
    if (peeked < 4) return false;

    // 检查同步头
    if (peekBuf[0] != 0xA5 || peekBuf[1] != 0x5A) {
        // 同步失败，丢弃一个字节
        uint8_t dummy;
        rxQueue.pop(&dummy, 1);
        return false;
    }

    uint8_t packet_type = peekBuf[2];
    uint8_t count = peekBuf[3];

    if (packet_type != CMD_REF_PARAMS || count != 3) {
        // 类型不对，丢弃整帧？或继续找头
        // 简单起见：丢弃一个字节
        uint8_t dummy;
        rxQueue.pop(&dummy, 1);
        return false;
    }

    // 计算完整帧长度
    size_t frameLen = 2 + 1 + 1 + 4 + 12 + 2; // = 22
    if (rxQueue.available() < frameLen) return false;

    // 读取整帧
    uint8_t frame[22];
    rxQueue.pop(frame, frameLen);

    // 校验 CRC
    uint16_t received_crc = *(uint16_t *) (frame + 20); // 小端
    uint16_t computed_crc = crc16(frame, 20);
    if (computed_crc != received_crc) {
        return false; // CRC 错误
    }

    // 解析数据
    float *data = reinterpret_cast<float *>(frame + 8); // offset: 2+1+1+4=8
    refParam.refSpeed = data[0];
    refParam.refPosition = data[1];
    refParam.refCurrent = data[2];

    return true; // 成功解析
}

