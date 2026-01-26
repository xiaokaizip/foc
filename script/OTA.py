import serial
import os
import time
from crcmod import crcmod

# === 常量 ===
SOH = 0x01      # 128-byte packet (rarely used in YMODEM)
STX = 0x02      # 1024-byte packet (standard for YMODEM)
EOT = 0x04
ACK = 0x06
NAK = 0x15
CAN = 0x18
EOF_PAD = 0x1A

def calc_crc16_xmodem(data: bytes) -> int:
    """与 C 函数完全等效的 CRC-16/XMODEM 实现"""
    crc = 0
    len = 0
    for byte in data:
        len = len +1
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    print(len)
    return crc

# === CRC-16/XMODEM (used by YMODEM) ===
crc16_func = crcmod.mkCrcFun(0x11021, initCrc=0x0000, rev=False, xorOut=0x0000)

def calc_crc(data: bytes) -> bytes:
    crc = calc_crc16_xmodem(data)
    print(crc)
    return crc.to_bytes(2, 'big')

def build_packet(packet_type: int, seq: int, data: bytes) -> bytes:
    """构建 YMODEM 数据包"""
    if packet_type == SOH:
        assert len(data) == 128
    elif packet_type == STX:
        assert len(data) == 1024
    else:
        raise ValueError("Invalid packet type")

    seq_byte = seq & 0xFF
    seq_inv = (~seq) & 0xFF
    header = bytes([packet_type, seq_byte, seq_inv])
    payload = header + data
    return payload + calc_crc(payload)

def send_ymodem_file(port: str, baudrate: int, filepath: str, timeout=100):
    if not os.path.isfile(filepath):
        print(f"❌ 文件不存在: {filepath}")
        return False

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"� 打开串口 {port} @ {baudrate} bps")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return False

    file_size = os.path.getsize(filepath)
    filename = os.path.basename(filepath).encode('ascii', errors='replace')

    try:
        # === 步骤1: 等待接收方发送 'C' ===
        print("⏳ 等待设备发送 'C' (YMODEM-CRC 请求)...")
        start = time.time()
        while time.time() - start < timeout:
            if ser.in_waiting:
                c = ser.read(1)
                if c == b'C':
                    print("✅ 收到 'C'，开始传输")
                    break
            time.sleep(0.1)
        else:
            print("❌ 超时：未收到 'C'")
            ser.close()
            return False

        # === 步骤2: 发送头帧（包号 = 0）===
        # 格式: "filename\0file_size\0"
        header_str = f"{filename.decode()}\0{file_size}\0".encode('ascii')
        if len(header_str) > 128:
            print("⚠️ 文件名或路径过长，可能被截断")
        header_data = header_str.ljust(128, b'\x1a')[:128]

        head_frame = build_packet(SOH, 0, header_data)
        ser.write(head_frame)
        print(f"� 发送头帧: {filename.decode()} ({file_size} bytes)")
        print(head_frame)
        # === 步骤3: 等待 ACK + 'C' ===
        responses = []
        start = time.time()
        print("等待回应")


        while time.time() - start < timeout and len(responses) < 2:
            if ser.in_waiting:
                responses.append(ser.read(1))
        if len(responses) >= 2 and responses[0] == b'\x06' and responses[1] == b'C':

            print("✅ 收到 ACK + 'C'，开始传数据")
        else:
            print(f"❌ 未收到预期响应，收到: {[r.hex() for r in responses]}")
            ser.close()
            return False



        # === 步骤4: 发送数据帧（1024 字节/帧）===
        with open(filepath, 'rb') as f:
            seq = 0
            while True:
                chunk = f.read(128)
                if not chunk:
                    break

                # 填充至 1024 字节
                if len(chunk) < 128:
                    chunk = chunk.ljust(128, b'\x1A')[:128]

                frame = build_packet(SOH, seq, chunk)
                print(frame)
                retry = 0
                while retry < 3:
                    ser.write(frame)
                    print(f"� 发送数据帧 #{seq} ({len(chunk)} bytes)")

                    # 等待 ACK
                    start_ack = time.time()
                    ack_received = False
                    while time.time() - start_ack < 10:
                        if ser.in_waiting:
                            resp = ser.read(1)
                            print(resp)
                            if resp == b'\x06':  # ACK
                                ack_received = True
                                break
                            elif resp == b'\x15':  # NAK
                                print(f"⚠️ 帧 #{seq} 收到 NAK，重试...")
                                retry += 1
                                break
                    if ack_received:
                        break
                    else:
                        retry += 1
                if retry >= 3:
                    print(f"❌ 帧 #{seq} 重试失败")
                    ser.close()
                    return False

                seq = (seq + 1) & 0xFF  # 循环 0-255

        # === 步骤5: 发送 EOT ===
        chunk = bytes([EOT])+bytes([EOT])
        if len(chunk) < 128:
                    chunk = chunk.ljust(128, b'\x1A')[:128]
        frame = build_packet(SOH, seq, chunk)
        ser.write(frame)
        print("� 发送 EOT")
        print(frame)

        # 等待 ACK
        start = time.time()
        while time.time() - start < timeout:
            if ser.in_waiting:
                if ser.read(1) == b'\x06':
                    print("✅ 收到 EOT 的 ACK")
                    break
        else:
            print("❌ 未收到 EOT 的 ACK")
            ser.close()
            return False

        # === 步骤6: 发送空头帧（包号=0）表示结束 ===
        empty_header = b'\x00' * 128
        end_frame = build_packet(SOH, 0, empty_header)
        ser.write(end_frame)
        print("� 发送结束帧（空头帧）")

        # 等待最终 ACK
        start = time.time()
        while time.time() - start < timeout:
            if ser.in_waiting:
                if ser.read(1) == b'\x06':
                    print("✅ 传输完成！")
                    ser.close()
                    return True
        print("❌ 未收到结束帧的 ACK")

    except Exception as e:
        print(f"� 传输异常: {e}")
    finally:
        ser.close()

    return False


# === 主程序入口 ===
if __name__ == "__main__":
    PORT = "COM3"
    BAUDRATE = 115200
    FILEPATH = "BLDC.bin"

    success = send_ymodem_file(PORT, BAUDRATE, FILEPATH)
    if success:
        print("� YMODEM 传输成功！")
    else:
        print("❌ YMODEM 传输失败！")