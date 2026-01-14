import numpy as np
import matplotlib.pyplot as plt

    # 基本参数
PP = 7  # 极对数
L = 1.67e-6  # 相电感
R = 2.25     # 相电阻
V_bus = 24   # 母线电压
I = 1.1      # 额定电流
I_peak = 3.2 # 峰值电流
f_sw = 10e3  # 开关频率

def calculate_dcdc_parameters():
    """
    DC-DC变换器参数计算与标准电感值选择
    """


    # DC-DC变换器参数
    DCDC_V_F = 0.55        # 二极管导通时的压降
    DCDC_VM = 24          # DC-DC输入电压
    DCDC_VOUT = 10        # DC-DC输出电压
    DCDC_f_sw = 500e3     # DC-DC开关频率
    DCDC_T = 1 / DCDC_f_sw  # 开关周期
    DCDC_I = 1.2          # DC-DC输出额定电流
    DCDC_inductance_ripple = 0.3

    # 计算占空比
    DCDC_duty = (DCDC_VOUT + DCDC_V_F) / (DCDC_VM + DCDC_V_F)
    
    # 假设额定电流为1A，电感电流纹波系数为0.4，则电感峰值电流为1.4A，峰峰值为0.8A
    # 计算所需电感值
    required_inductance = (DCDC_VM - DCDC_VOUT) * DCDC_duty * DCDC_T / (DCDC_inductance_ripple*2 * DCDC_I)
    
    # 标准电感值系列（单位：μH）
    standard_inductances = [1, 2.2, 3.3, 4.7, 6.8, 10, 15, 22, 33, 47]
    
    # 找到最接近的标准电感值
    closest_value = min(standard_inductances, key=lambda x: abs(x - required_inductance * 1e6))
    
    # 计算使用标准值后的实际电感电流
    actual_inductance = closest_value * 1e-6  # 转换为亨利
    actual_inductance_peak_current = (DCDC_VM - DCDC_VOUT) / actual_inductance * DCDC_duty * (1 / DCDC_f_sw)

    actual_max_current = DCDC_I + actual_inductance_peak_current/2

    #电感选取   增益    的ZEPIM322520S-150M，封装为1210的电感，感值为15uH，电流为2.1A，内阻最大为260毫欧，立创上的编号为C48945853
    #二极管选取的最大电流和电感的最大电流相同，所以在选取的时候，可以选取2A电流的二极管，选取安森美的SS24FL，封装为SOD-123F，压降为550mV，正向电流为2A。C894433
    
    #电容的取值按照给出的原理图来给 输入电容为22uF+100nF，输出为两个22uF 25V的电容，这些电容在立创上都能免费贴片
    DCDC_CIN = 22e-6
    DCDC_COUT = 22e-6*2
    print("DC-DC变换器参数计算结果：")
    print(f"输入电压: {DCDC_VM}V")
    print(f"输出电压: {DCDC_VOUT}V")
    print(f"占空比: {DCDC_duty:.3f}")
    print(f"开关频率: {DCDC_f_sw/1000}kHz")
    print(f"所需电感值: {required_inductance*1e6:.2f}μH")
    print(f"最接近的标准电感值: {closest_value}μH")
    print(f"使用标准电感值后的电感最大电流: {actual_max_current:.3f}A")
    
    return {
        "input_voltage": DCDC_VM,
        "output_voltage": DCDC_VOUT,
        "duty_cycle": DCDC_duty,
        "switch_freq": DCDC_f_sw,
        "required_inductance": required_inductance,
        "standard_inductance": actual_inductance,
        "inductor_current": actual_max_current
    }

def calculate_driver_parameters():
    print("\n" + "="*50)
    #驱动的参数
    I_source = 1.3     #灌电流和拉电流的峰值
    I_sink = 1.3

    R_PMOS = 3.7            #上管MOS的内阻，即在开启MOS时的内阻
    R_NMOS = 1.6            

    Dead_times = 150e-9         #驱动的死区时间 单位是秒

    t_on = 60e-9                #驱动的上升沿和下降沿的时间为

    #采样电流，电源电压为24v.而相电阻为2.25欧，无论如何电机的电流都不会超过24/4.5 = 5.3A ,所以在设计的时候采样电流按照最大5.3A设计
    #电阻采用50毫欧的电阻，峰值电流为2.1A，电阻上的最大功耗为 2.1*2.1*0.05 = 0.21W，选择0805封装的采样电阻即可，放大倍数为5倍


def clark_transformation(Ua, Ub, Uc):
    """
    进行 Clark 变换，将三相 abc 转换为两相 αβ。
    :param Ua: A 相电压或电流
    :param Ub: B 相电压或电流
    :param Uc: C 相电压或电流
    :return: αβ 两相分量
    """
    alpha = Ua
    beta = (1 / np.sqrt(3)) * Ub - (1 / np.sqrt(3)) * Uc
    return alpha, beta

def park_transformation(alpha, beta, theta):
    """
    进行 Park 变换，将 αβ 静止坐标系转换为旋转 dq 坐标系。
    :param alpha: α 分量
    :param beta: β 分量
    :param theta: 与转子同步的角度
    :return: d 和 q 分量
    """
    d = alpha * np.cos(theta) + beta * np.sin(theta)
    q = -alpha * np.sin(theta) + beta * np.cos(theta)
    return d, q

def plot_signals(t, signals, labels, title):
    """
    绘制信号图。
    :param t: 时间轴
    :param signals: 信号列表
    :param labels: 对应信号的标签列表
    :param title: 图表标题
    """
    plt.figure()
    for signal, label in zip(signals, labels):
        plt.plot(t, signal, label=label)
    plt.title(title)
    plt.xlabel('Angle (rad)')
    plt.ylabel('Amplitude')
    plt.legend()
    plt.grid(True)

def foc():
    # 时间/角度轴
    # 参数设置
    freq = 10e3  # 10kHz 频率
    sample_rate = 100e3  # 假设采样率为 100kHz
    duration = 0.05  # 持续时间为 2ms
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)  # 时间轴



    # 生成三相交流信号（保持原样）
    angle_range = (0, 2 * np.pi)
    num_points = len(t)
    theta = np.linspace(angle_range[0], angle_range[1], num_points)

    # 三相电压信号 (假设频率与转子同步)
    Ua = np.cos(theta)
    Ub = np.cos(theta - 2*np.pi/3)
    Uc = np.cos(theta + 2*np.pi/3)
    U = Ua +Ub +Uc

    Uab = Ua - Ub
    Uac = Ua - Uc
    Ubc = Ub - Uc

    # ✅ 关键：Park 变换使用与信号同步的旋转角度 θ
    # 假设转子以相同速度旋转，θ = t
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # # Park Transformation (abc -> αβ -> dq 或直接 abc -> dq)
    # # 使用标准变换矩阵
    d = (2/3) * (Ua * cos_theta +
                 Ub * np.cos(theta - 2*np.pi/3) +
                 Uc * np.cos(theta + 2*np.pi/3))

    q = (2/3) * (Ua * (-sin_theta) +
                 Ub * (-np.sin(theta - 2*np.pi/3)) +
                 Uc * (-np.sin(theta + 2*np.pi/3)))


    # 生成 10kHz 方波信号
    square_wave = np.sign(np.sin(2 * np.pi * freq * t))

    # 绘制原始三相信号
    plot_signals(t, [Ua, Ub, Uc,U], ['Ua', 'Ub', 'Uc','U'], 'Three-Phase Signals')

    plot_signals(t, [Uab, Ubc, Uac], ['Uab', 'Ubc', 'Ubc'], 'Three-Phase Signals')

    # 绘制 Park 变换后的 dq 信号
    plot_signals(t, [d, q], ['d-axis', 'q-axis'], 'Park Transformation Results')

    
    # 绘制生成的 10kHz 方波信号
    plt.figure()
    plt.plot(t, square_wave, label='10kHz Square Wave')
    plt.title('10kHz Square Wave Signal')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.legend()
    plt.grid(True)

    plt.show()




    # 打印平均值，验证是否为直流
    print(f"d 平均值: {np.mean(d):.4f} ± {np.std(d):.4f}")
    print(f"q 平均值: {np.mean(q):.4f} ± {np.std(q):.4f}")

def select_standard_component(value, standard_series, unit=""):
    """
    从标准系列中选择最接近的值
    
    Args:
        value: 需要匹配的值
        standard_series: 标准值系列
        unit: 单位字符串
    
    Returns:
        dict: 包含选择结果的字典
    """
    closest_value = min(standard_series, key=lambda x: abs(x - value))
    return {
        "calculated_value": value,
        "selected_value": closest_value,
        "difference": abs(closest_value - value),
        "relative_error": abs(closest_value - value) / value * 100 if value != 0 else 0
    }

def main():
    # 执行DC-DC参数计算
    results = calculate_dcdc_parameters()
    
    print("\n" + "="*50)

    
    # 示例：选择标准电阻值
    standard_resistors = [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 
                         3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1] + \
                         [x*10 for x in [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 
                         3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1]] + \
                         [x*100 for x in [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 
                         3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1]]
    

if __name__ == "__main__":
    main()
    foc()


