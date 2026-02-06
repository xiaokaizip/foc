import serial
import struct
import threading
import time
from collections import deque

# ====== 配置 ======
SERIAL_PORT = 'COM3'        # � 改成你的串口，Linux 用 '/dev/ttyUSB0'
BAUDRATE = 115200
MAX_FLOATS = 16

# 协议标识
FULL_HEADER = [0xA5, 0x5A]
SIMPLE_MAGIC = [0x00, 0x00, 0x80, 0x7F]
INIT_HEADER = [0xAA, 0x55]

stop_event = threading.Event()
raw_queue = deque()
parsed_frames = deque()

def crc16_ccitt(data: bytes) -> int:
    """与 STM32 一致的 CRC16-CCITT (初始值 0xFFFF, 多项式 0x1021)"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
    return crc & 0xFFFF

def parse_buffer(buffer: bytearray):
    results = []
    while len(buffer) >= 10:
        # === 1. 尝试完整协议 (A5 5A ...) ===
        if len(buffer) >= 2 and buffer[0] == FULL_HEADER[0] and buffer[1] == FULL_HEADER[1]:
            if len(buffer) < 4:
                break
            packet_type = buffer[2]
            count = buffer[3]
            if not (1 <= count <= MAX_FLOATS):
                buffer = buffer[1:]
                continue

            expected_len = 10 + 4 * count  # 4(header)+4(ts)+4*count+2(crc)
            if len(buffer) < expected_len:
                break

            frame = buffer[:expected_len]
            calc_crc = crc16_ccitt(frame[:-2])
            rx_crc = struct.unpack('<H', frame[-2:])[0]
            if calc_crc != rx_crc:
                buffer = buffer[1:]
                continue

            ts = struct.unpack('<I', frame[4:8])[0]  # uint32_t timestamp
            floats = [struct.unpack('<f', frame[8 + i*4 : 8 + (i+1)*4])[0] for i in range(count)]

            results.append({
                'type': 'full',
                'timestamp': ts,
                'packet_type': packet_type,
                'data': floats
            })
            buffer = buffer[expected_len:]
            continue

        # === 2. 尝试简化协议 (00 00 80 7F ...) ===
        if len(buffer) >= 4 and list(buffer[:4]) == SIMPLE_MAGIC:
            if len(buffer) < 12:  # 至少要有 count + ts + 1 float
                break

            # 读取 count (as float)
            count_float = struct.unpack('<f', buffer[4:8])[0]
            count = int(round(count_float))
            if not (1 <= count <= MAX_FLOATS):
                buffer = buffer[1:]
                continue

            expected_len = 12 + 4 * count  # 4(magic)+4(count)+4(ts)+4*count
            if len(buffer) < expected_len:
                break

            ts_float = struct.unpack('<f', buffer[8:12])[0]
            ts = int(round(ts_float))
            floats = [struct.unpack('<f', buffer[12 + i*4 : 12 + (i+1)*4])[0] for i in range(count)]

            results.append({
                'type': 'simple',
                'timestamp': ts,
                'data': floats
            })
            buffer = buffer[expected_len:]
            continue

        # === 3. 初始化包 (AA 55 ...) ===
        if len(buffer) >= 4 and list(buffer[:2]) == INIT_HEADER:
            time_unit = buffer[2]  # 假设第3字节是时间单位（ms）
            results.append({
                'type': 'init',
                'time_unit_ms': time_unit
            })
            buffer = buffer[4:]
            continue

        # 未知数据，跳过首字节以恢复同步
        buffer = buffer[1:]

    return results, buffer

# ====== 串口接收线程 ======
def serial_reader():
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"[RX] Listening on {SERIAL_PORT} @ {BAUDRATE}")
            while not stop_event.is_set():
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    raw_queue.append(data)
                else:
                    time.sleep(0.001)
    except Exception as e:
        print(f"[RX ERROR] {e}")

# ====== 解析线程 ======
def parser_worker():
    aggregated = bytearray()
    last_parse = time.time()
    
    while not stop_event.is_set():
        while raw_queue:
            aggregated.extend(raw_queue.popleft())
        
        now = time.time()
        # 触发条件：数据足够多 或 超时
        if len(aggregated) >= 20 or (len(aggregated) >= 10 and now - last_parse > 0.01):
            frames, aggregated = parse_buffer(aggregated)
            for frame in frames:
                parsed_frames.append(frame)
            last_parse = now
        
        # 防止内存爆炸
        if len(aggregated) > 500:
            aggregated.clear()
        
        time.sleep(0.001)

# ====== 主循环 ======
def main():
    rx_thread = threading.Thread(target=serial_reader, daemon=True)
    parse_thread = threading.Thread(target=parser_worker, daemon=True)
    rx_thread.start()
    parse_thread.start()

    print("[MAIN] Ready. Parsing frames...")
    try:
        while True:
            while parsed_frames:
                frame = parsed_frames.popleft()
                if frame['type'] == 'init':
                    print(f"[INIT] Time unit: {frame['time_unit_ms']} ms")
                elif frame['type'] == 'full':
                    print(f"[FULL] t={frame['timestamp']} ms, type=0x{frame['packet_type']:02X}, "
                          f"data={frame['data']}")
                elif frame['type'] == 'simple':
                    print(f"[SIMPLE] t={frame['timestamp']} ms, data={frame['data']}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
        stop_event.set()
        rx_thread.join(timeout=1)
        parse_thread.join(timeout=1)

if __name__ == '__main__':
    main()