// ring_buffer.h
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstdint>
#include <cstddef>

class RingBuffer {
public:
    RingBuffer(size_t capacity);

    ~RingBuffer();

    bool push(const uint8_t *data, size_t len);

    size_t pop(uint8_t *buffer, size_t max_len); // 返回实际弹出长度
    size_t available() const;

    size_t freeSpace() const;

    size_t peek(uint8_t *out, size_t max_len) const;

    void reset();

private:
    volatile size_t head;
    volatile size_t tail;
    size_t capacity;
    uint8_t buffer[1024];
};


#endif
