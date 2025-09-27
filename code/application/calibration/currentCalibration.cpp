// currentCalibration.cpp
#include "currentCalibration.h"

int readCurrentA() {
    /* return ADC_Read(IA); */
    return 6;
} // 示例
int readCurrentB() {
    /* return ADC_Read(IB); */
    return -3;
}

int readCurrentC() {
    /* return ADC_Read(IC); */
    return 4;
}

int currentCalibration::calculateAverage(int arr[10]) {
    int sum = 0;
    int min_val = arr[0], max_val = arr[0];

    for (int i = 0; i < 10; ++i) {
        sum += arr[i];
        if (arr[i] < min_val) min_val = arr[i];
        if (arr[i] > max_val) max_val = arr[i];
    }

    sum = sum - min_val - max_val; // 去掉最大和最小
    return sum / 8; // 对剩余 8 个求平均
}

void currentCalibration::calibrate(float period) {
    // 已校准则不再执行
    if (calibration_done) {
        return;
    }

    // tick_count 每 1ms 加 1
    tick_count++;

    // 每 10ms（即 tick_count % 10 == 0）进行一次有效采样
    if (tick_count % 10 == 0) {
        // 仅在 sample_count < 10 时采样（最多采 10 次）
        if (sample_count < 10) {
            currentA[sample_count] = current[0];
            currentB[sample_count] = current[1];
            currentC[sample_count] = current[2];

            sample_count++;
        }

        // 如果已经采集满 10 次（即 10 × 10ms = 100ms）
        if (sample_count >= 10) {
            // 计算三相偏移量
            current_offset[0] = static_cast<float>(calculateAverage(currentA));
            current_offset[1] = static_cast<float>(calculateAverage(currentB));
            current_offset[2] = static_cast<float>(calculateAverage(currentC));

            // 标记状态
            calibrated = true;
            calibration_done = true;

            // 可选：调试打印
            // printf("Calibration Done! Offsets: [%d, %d, %d]\n",
            //        current_offset[0], current_offset[1], current_offset[2]);
        }
    }
}
