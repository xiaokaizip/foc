import serial
import struct
import threading
import time
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ====== 配置 ======
SERIAL_PORT = 'COM3'        # ← 修改为你的串口
BAUDRATE = 115200
MAX_FLOATS = 16
MAX_PLOT_POINTS = 200  # 最多显示多少个点

FULL_HEADER = [0xA5, 0x5A]
CMD_REF_PARAMS = 0x10
CRC16_POLY = 0x1021

# 全局
stop_event = threading.Event()
raw_queue = deque()
parsed_frames = deque()
ser = None

# 绘图数据（线程安全：只在主线程读写）
plot_data = {i: {'x': [], 'y': []} for i in range(MAX_FLOATS)}  # 每个通道独立存储

# ==================== CRC & 发送 ====================
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ CRC16_POLY
            else:
                crc <<= 1
    return crc & 0xFFFF

def send_ref_params(ref_speed: float, ref_position: float, ref_current: float):
    global ser
    if ser is None or not ser.is_open:
        print("[TX] 串口未打开！")
        return
    count = 3
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF
    frame = bytearray()
    frame.extend(FULL_HEADER)
    frame.append(CMD_REF_PARAMS)
    frame.append(count)
    frame.extend(struct.pack('<I', timestamp))
    frame.extend(struct.pack('<f', ref_speed))
    frame.extend(struct.pack('<f', ref_position))
    frame.extend(struct.pack('<f', ref_current))
    crc = crc16_ccitt(frame)
    frame.extend(struct.pack('<H', crc))
    try:
        ser.write(frame)
        print(f"[TX] Sent: speed={ref_speed:.3f}, pos={ref_position:.3f}, current={ref_current:.3f}")
    except Exception as e:
        print(f"[TX ERROR] {e}")

# ==================== 协议解析 ====================
def parse_buffer(buffer: bytearray):
    results = []
    i = 0
    while i < len(buffer):
        if i + 2 <= len(buffer) and buffer[i] == FULL_HEADER[0] and buffer[i+1] == FULL_HEADER[1]:
            if i + 4 > len(buffer):
                break
            packet_type = buffer[i+2]
            count = buffer[i+3]
            if not (1 <= count <= MAX_FLOATS):
                i += 1
                continue
            expected_len = 10 + 4 * count
            if i + expected_len > len(buffer):
                break
            frame = buffer[i:i+expected_len]
            calc_crc = crc16_ccitt(frame[:-2])
            rx_crc = struct.unpack('<H', frame[-2:])[0]
            if calc_crc != rx_crc:
                i += 1
                continue
            ts = struct.unpack('<I', frame[4:8])[0]
            floats = [struct.unpack('<f', frame[8 + j*4 : 8 + (j+1)*4])[0] for j in range(count)]
            results.append({'type': 'full', 'timestamp': ts, 'packet_type': packet_type, 'data': floats})
            i += expected_len
            continue
        i += 1
    return results, buffer[i:] if i < len(buffer) else bytearray()

# ==================== 串口线程 ====================
def serial_reader():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"[RX] 已连接 {SERIAL_PORT}")
        while not stop_event.is_set():
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                # print("RAW RX:", data.hex())  # 调试用
                raw_queue.append(data)
            else:
                time.sleep(0.001)
    except Exception as e:
        print(f"[RX ERROR] {e}")
    finally:
        if ser and ser.is_open:
            ser.close()

def parser_worker():
    aggregated = bytearray()
    while not stop_event.is_set():
        while raw_queue:
            aggregated.extend(raw_queue.popleft())
        if len(aggregated) >= 10:
            frames, aggregated = parse_buffer(aggregated)
            for frame in frames:
                parsed_frames.append(frame)
        time.sleep(0.005)

# ==================== 主窗口 ====================
class MainWindow:
    def __init__(self, root):
        self.root = root
        root.title("STM32 实时数据监控")
        root.geometry("1000x700")

        # === 上半部分：发送控制 ===
        control_frame = tk.LabelFrame(root, text="发送控制", padx=10, pady=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        tk.Label(control_frame, text="Ref Speed:").grid(row=0, column=0, padx=5)
        self.speed_var = tk.StringVar(value="0.0")
        tk.Entry(control_frame, textvariable=self.speed_var, width=10).grid(row=0, column=1, padx=5)

        tk.Label(control_frame, text="Ref Position:").grid(row=0, column=2, padx=5)
        self.pos_var = tk.StringVar(value="0.0")
        tk.Entry(control_frame, textvariable=self.pos_var, width=10).grid(row=0, column=3, padx=5)

        tk.Label(control_frame, text="Ref Current:").grid(row=0, column=4, padx=5)
        self.current_var = tk.StringVar(value="0.0")
        tk.Entry(control_frame, textvariable=self.current_var, width=10).grid(row=0, column=5, padx=5)

        tk.Button(control_frame, text="发送参数", command=self.on_send,
                  bg="#4CAF50", fg="white").grid(row=0, column=6, padx=10)

        # === 下半部分：绘图区 ===
        plot_frame = tk.LabelFrame(root, text="实时曲线", padx=10, pady=10)
        plot_frame.pack(fill='both', expand=True, padx=10, pady=5)

        # 选择通道
        selector_frame = tk.Frame(plot_frame)
        selector_frame.pack(anchor='w', pady=5)
        tk.Label(selector_frame, text="绘制通道:").pack(side='left')
        self.channel_var = tk.StringVar(value="0")
        channel_choices = [str(i) for i in range(MAX_FLOATS)]
        self.channel_menu = ttk.Combobox(selector_frame, textvariable=self.channel_var, values=channel_choices, width=5, state="readonly")
        self.channel_menu.pack(side='left', padx=5)

        # Matplotlib 图形
        self.fig = Figure(figsize=(8, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("等待数据...")
        self.ax.set_xlabel("时间戳 (ms)")
        self.ax.set_ylabel("数值")
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # 启动轮询
        self.root.after(50, self.poll_and_update)

    def on_send(self):
        try:
            speed = float(self.speed_var.get())
            pos = float(self.pos_var.get())
            current = float(self.current_var.get())
            threading.Thread(target=send_ref_params, args=(speed, pos, current), daemon=True).start()
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字！")

    def poll_and_update(self):
        """主线程：消费帧 + 更新绘图"""
        # 1. 消费接收到的帧，更新 plot_data
        while parsed_frames:
            frame = parsed_frames.popleft()
            if frame['type'] == 'full':
                ts = frame['timestamp']
                data = frame['data']
                for idx, val in enumerate(data):
                    plot_data[idx]['x'].append(ts)
                    plot_data[idx]['y'].append(val)
                    # 限制长度
                    if len(plot_data[idx]['x']) > MAX_PLOT_POINTS:
                        plot_data[idx]['x'].pop(0)
                        plot_data[idx]['y'].pop(0)

        # 2. 更新图形
        try:
            channel = int(self.channel_var.get())
            if channel < 0 or channel >= MAX_FLOATS:
                channel = 0
        except:
            channel = 0

        x = plot_data[channel]['x']
        y = plot_data[channel]['y']

        self.ax.clear()
        if x:
            self.ax.plot(x, y, '-o', markersize=3)
            self.ax.set_title(f"通道 {channel} 实时数据")
            self.ax.set_xlabel("时间戳 (ms)")
            self.ax.set_ylabel("数值")
            self.ax.grid(True)
            # 自动调整 x 轴范围
            if len(x) > 1:
                x_range = x[-1] - x[0]
                if x_range > 0:
                    self.ax.set_xlim(x[0], x[-1])
        else:
            self.ax.set_title(f"通道 {channel}：无数据")
            self.ax.text(0.5, 0.5, '等待数据...', horizontalalignment='center', verticalalignment='center', transform=self.ax.transAxes)

        self.canvas.draw()

        # 继续轮询
        self.root.after(100, self.poll_and_update)  # 每100ms更新一次

# ==================== 主程序 ====================
def main():
    root = tk.Tk()
    app = MainWindow(root)

    threading.Thread(target=serial_reader, daemon=True).start()
    threading.Thread(target=parser_worker, daemon=True).start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if ser and ser.is_open:
            ser.close() 

if __name__ == '__main__':
    main()