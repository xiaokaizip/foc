import serial
import os
import time
import threading
from crcmod.predefined import Crc

# # ===== 全局状态 =====
# stop_all = False
# start_ymodem = False  # 触发 YMODEM 的标志

# ===== CRC 计算（与你之前一致）=====
def calc_crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    return crc

def calc_crc(data: bytes) -> bytes:
    return calc_crc16_xmodem(data).to_bytes(2, 'big')

def build_packet(packet_type: int, seq: int, data: bytes) -> bytes:
    if packet_type == 0x01:  # SOH
        assert len(data) == 128
    elif packet_type == 0x02:  # STX
        assert len(data) == 1024
    else:
        raise ValueError("Invalid packet type")
    header = bytes([packet_type, seq & 0xFF, (~seq) & 0xFF])
    payload = header + data

    crc_func = Crc('xmodem')
    crc_func.update(data)
    crc_bytes = crc_func.crcValue.to_bytes(2, 'big')


    return payload + crc_bytes

# ===== 串口接收线程 =====
def receive_and_printf(ser):
    while not stop_all:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            try:
                decoded = data.decode('ascii', errors='replace')
                print(f"← RX: {repr(decoded)}")
            except:
                print(f"← RX (hex): {data.hex()}")
        time.sleep(0.05)

# ===== 键盘输入线程 =====
def keyboard_input(ser):
    global start_ymodem
    print("⌨️ 请输入命令 (1=触发升级):")
    while not stop_all:
        try:
            ser.reset_input_buffer()
            print("� 清空串口缓冲区")
            user_input = input().strip()
            if user_input == '1':
                ser.write(b'1')
                start_ymodem = True  # ✅ 全局标志
                print("✅ 已发送 '1'，等待设备响应...")
            if user_input == '3':
                ser.write(b'3')

            else:
                print(f"⏭️ 忽略: '{user_input}'")
        except Exception as e:
            if not stop_all:
                print(f"⚠️ 输入错误: {e}")
            break

# ===== YMODEM 传输函数（简化版）=====
def send_ymodem_file(ser, filepath, timeout=10):
    if not os.path.isfile(filepath):
        print(f"❌ 文件不存在: {filepath}")
        return False

    file_size = os.path.getsize(filepath)
    filename = os.path.basename(filepath).encode('ascii', errors='replace')
    
    # === 等待 'C' ===
    print("⏳ 等待设备发送 'C'...")
    while True:
        if ser.read(1)==b'C':
            break

    # === 发送头帧 ===
    header_str = f"{filename.decode()}\0{file_size}\0".encode('ascii')
    header_data = header_str.ljust(128, b'\x1A')
    head_frame = build_packet(0x01, 0, header_data)
    ser.write(head_frame)
    print(f"� 发送头帧: {filename.decode()}")

    # ✅ 正确：先等 ACK，再等 'C'
    print("⏳ 等待 ACK...")
    start_time = time.time()
    timeout = 2.0
    c_char=b''
    while time.time() - start_time <timeout:
        if ser.in_waiting>0:
            c_char = ser.read(1)
            if c_char == b'\06':
                break
        time.sleep(0.01)
    

    print("✅ 收到 ACK，等待 'C'...")
    start_time = time.time()
    timeout = 2.0
    c_char=b''
    while time.time() - start_time <timeout:
        if ser.in_waiting>0:
            c_char = ser.read(1)
            if c_char == b'C':
                break
        time.sleep(0.01)


    print("✅ 收到 'C'，开始发送数据帧")

    # 5. 发送数据帧（SOH, 128B）
    with open(filepath, 'rb') as f:
        seq = 1
        while True:
            chunk = f.read(128)
            if not chunk:
                break
            chunk = chunk.ljust(128, b'\x1A')
            frame = build_packet(0x01, seq, chunk)
            print(frame)
            ser.write(frame)

            # 等待 ACK
            print("⏳ 等待 ACK...")
            start_time = time.time()
            timeout = 2.0
            c_char=b''
            while time.time() - start_time <timeout:
                if ser.in_waiting>0:
                    c_char = ser.read(1)
                if c_char == b'\06':
                    break
            time.sleep(0.01)
            if c_char != b'\x06':
                print(f"❌ 数据帧 #{seq} 未收到 ACK，收到: {c_char}")
                return False
            seq = (seq + 1) % 256

    # 6. 发送 EOT
    ser.write(b'\x04')
                # 等待 ACK
    print("⏳ 等待回复结束")
    start_time = time.time()
    timeout = 2.0
    c_char=b''
    while time.time() - start_time <timeout:
        if ser.in_waiting>0:
            c_char = ser.read(1)
        if c_char == b'\06':
            break
    time.sleep(0.01)
    if c_char != b'\x06':
        print(f"❌ 数据帧 #{seq} 未收到 ACK，收到: {c_char}")
        return False

    # 7. 发送空头帧
    empty = b'\x00' * 128
    frame_end = build_packet(0x01, 0, empty)
    print("发送空头包")
    ser.write(frame_end)
    print("⏳ 等待回复空头包")
    start_time = time.time()
    timeout = 2.0
    c_char=b''
    while time.time() - start_time <timeout:
        if ser.in_waiting>0:
            c_char = ser.read(1)
        if c_char == b'\06':
            break
    time.sleep(0.01)
    if c_char != b'\x06':
        print(f"❌ 数据帧 #{seq} 未收到 ACK，收到: {c_char}")
        return False
    print("下载完成")

# ===== 主程序 =====
if __name__ == "__main__":
    PORT = "COM3"
    BAUDRATE = 115200
    FILEPATH = "BLDC.bin"

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"� 打开串口 {PORT}")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        exit(1)

    global stop_all, start_ymodem
    stop_all = False
    start_ymodem = False

    # 启动监听线程
    recv_thread = threading.Thread(target=receive_and_printf, args=(ser,), daemon=True)
    input_thread = threading.Thread(target=keyboard_input, args=(ser,), daemon=True)
    recv_thread.start()
    input_thread.start()

    print("� 系统运行中... 输入 '1' 触发升级流程\n")

    try:
        while not stop_all:
            if start_ymodem:
                print("\n� 检测到升级请求，准备 YMODEM 传输...")
                stop_all = True  # 停止所有后台线程
                
                # 等待线程结束（最多1秒）
                recv_thread.join(timeout=1)
                input_thread.join(timeout=1)
                
                # 执行 YMODEM（此时串口由主线程独占）
                success = send_ymodem_file(ser, FILEPATH)
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n� 用户中断")
    finally:
        ser.close()
        print("� 串口已关闭")