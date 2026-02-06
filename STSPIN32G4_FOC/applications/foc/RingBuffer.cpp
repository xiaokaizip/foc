// ring_buffer.cpp
#include "RingBuffer.h"
#include <cstring>

RingBuffer::RingBuffer(size_t cap) : head(0), tail(0), capacity(cap + 1) {
    // buffer = new uint8_t[capacity];
}

RingBuffer::~RingBuffer() {
    // delete[] buffer;
}

bool RingBuffer::push(const uint8_t *data, size_t len) {
    if (len > freeSpace()) return false;
    size_t pos = head;
    for (size_t i = 0; i < len; ++i) {
        buffer[pos] = data[i];
        pos = (pos + 1) % capacity;
    }
    head = pos;
    return true;
}

size_t RingBuffer::pop(uint8_t *out, size_t max_len) {
    size_t avail = available();
    size_t toRead = (avail < max_len) ? avail : max_len;
    size_t pos = tail;
    for (size_t i = 0; i < toRead; ++i) {
        out[i] = buffer[pos];
        pos = (pos + 1) % capacity;
    }
    tail = pos;
    return toRead;
}

size_t RingBuffer::peek(uint8_t *out, size_t max_len) const {
    size_t avail = available();
    size_t toRead = (avail < max_len) ? avail : max_len;
    size_t pos = tail;
    for (size_t i = 0; i < toRead; ++i) {
        out[i] = buffer[pos];
        pos = (pos + 1) % capacity;
    }
    return toRead;
}

size_t RingBuffer::available() const {
    return (head >= tail) ? (head - tail) : (capacity - tail + head);
}

size_t RingBuffer::freeSpace() const {
    return capacity - 1 - available();
}

void RingBuffer::reset() {
    head = tail = 0;
}
