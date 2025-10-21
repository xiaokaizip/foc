// currentCalibration.h
#ifndef CURRENTCALIBRATION_H
#define CURRENTCALIBRATION_H

int readCurrentA(); // 替换为你的实际 ADC 读取函数
int readCurrentB();

int readCurrentC();

class currentCalibration {
public:
    float current_offset[3] = {0, 0, 0}; // 校准后的偏移量
    bool calibrated = false; // 是否已校准
    int current[3] = {0, 0, 0};

    void calibrate(float period); // 每 1ms 调用一次


private:
    unsigned int tick_count = 0; // 1ms 计数器（用于每 10ms 采样）
    unsigned int sample_count = 0; // 有效采样次数（0~9）

    int currentA[10] = {0}; // 存储 A 相 10 次有效采样
    int currentB[10] = {0}; // 存储 B 相
    int currentC[10] = {0}; // 存储 C 相

    bool calibration_done = false; // 防止重复执行

    // 去极值求平均
    int calculateAverage(int arr[10]);
};

#endif // CURRENTCALIBRATION_H
