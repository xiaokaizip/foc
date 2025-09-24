// position_sensor.cpp
#include "position_sensor.h"

#include <cmath>
#include <cstring>


extern "C" {
#include "AS5047.h"  // 假设 AS5047 驱动存在
}

#define PI 3.141592653589793f

// 全局实例
EncoderData_t encoderData = {0};

// 极对数和每转分辨率（来自 AS5047P）
const float POLE_PAIRS = 7.0f;
const int COUNTS_PER_REV = 16384; // CPR: 14-bit resolution
const float MECH_ANGLE_PER_COUNT = 2.0f * PI / COUNTS_PER_REV;

// ===== 私有状态变量（原全局静态变量）=====
namespace {
    bool first_run = true;
    uint16_t prev_raw_angle = 0;
    int total_rotations = 0; // 总机械圈数（用于累计角度）

    // 当前机械/电角度（连续值，含圈数）
    float mech_position = 0.0f; // 累积机械角度 (rad)
    float mod_mech_position = 0.0f; // mod 2π 的机械角度
    float old_mod_mech_position = 0.0f;

    // 速度滤波用
    float velVec[40] = {0};
    float velocity_filtered = 0.0f;

    // 滤波参数（可调）
    const float VELOCITY_LPF_ALPHA = 0.8f;
} // namespace


/**
 * @brief 更新编码器数据，使用类似 PositionSensorAM5147 的角度累积方式
 * @param devidx 设备索引
 * @param dt 时间间隔（秒）
 */
void Encoder_Update(int devidx, float dt) {
    // 读取当前原始角度（0~16383）
    uint16_t raw_angle = AS5047_ReadData(devidx, ANGLEUNC_AS5047P_VOL_REG_ADD);

    // 读取诊断信息
    uint16_t mag_value = AS5047D_Get_MAGCORDIC_Value(devidx);
    uint8_t agc_value = AS5047P_Get_AGC_Value(devidx);
    uint16_t error_flags = AS5047_Get_ERRFL(devidx);

    if (first_run) {
        prev_raw_angle = raw_angle;
        old_mod_mech_position = 0.0f;
        velocity_filtered = 0.0f;
        std::memset(velVec, 0, sizeof(velVec));
        first_run = false;
        return;
    }

    // ====== 1. 计算原始角度变化（处理跨零）======
    int raw_diff = static_cast<int>(raw_angle) - static_cast<int>(prev_raw_angle);

    // 处理跨 0 或跨 16383 的情况
    if (raw_diff > COUNTS_PER_REV / 2) {
        raw_diff -= COUNTS_PER_REV;
        total_rotations--; // 正向过零（从高到低），圈数减
    } else if (raw_diff < -COUNTS_PER_REV / 2) {
        raw_diff += COUNTS_PER_REV;
        total_rotations++; // 反向过零（从低到高），圈数加
    }

    // 更新累积机械角度（弧度）
    float delta_angle = raw_diff * MECH_ANGLE_PER_COUNT;
    mech_position += delta_angle;

    // 计算 mod 2π 的机械角度（用于速度计算）
    mod_mech_position = fmodf(mech_position, 2.0f * PI);
    if (mod_mech_position < 0.0f) mod_mech_position += 2.0f * PI;

    // ====== 2. 速度计算（使用 mod_mech_position 过零检测）======
    float angular_velocity = 0.0f;

    if (dt > 0.0f) {
        float d_theta = mod_mech_position - old_mod_mech_position;

        // 处理 mod 2π 跨越
        if (d_theta < -3.0f) {
            // ~ -π
            d_theta += 2.0f * PI;
        } else if (d_theta > 3.0f) {
            // ~ π
            d_theta -= 2.0f * PI;
        }

        angular_velocity = d_theta / dt;

        // 移动平均滤波 (N=40)
        int n = 40;
        float sum = angular_velocity;
        for (int i = 1; i < n; ++i) {
            velVec[n - i] = velVec[n - i - 1];
            sum += velVec[n - i];
        }
        velVec[0] = angular_velocity;
        angular_velocity = sum / static_cast<float>(n);

        // 一阶低通滤波
        angular_velocity = VELOCITY_LPF_ALPHA * angular_velocity +
                           (1.0f - VELOCITY_LPF_ALPHA) * velocity_filtered;
        velocity_filtered = angular_velocity;
    }

    // ====== 3. 计算电角度（电角度 = 机械角度 × 极对数 mod 2π）======
    float electrical_angle = fmodf(mech_position * POLE_PAIRS, 2.0f * PI);
    if (electrical_angle < 0.0f) electrical_angle += 2.0f * PI;

    // ====== 4. 累积角度（度）======
    float cumulative_angle_deg = mech_position; // rad → deg

    // ====== 5. 更新全局数据结构 ======
    encoderData.raw_value = raw_angle;
    encoderData.mechanical_angle = mod_mech_position; // [0, 2π)
    encoderData.electrical_angle = electrical_angle; // [0, 2π)
    encoderData.angular_velocity = angular_velocity; // rad/s (filtered)
    encoderData.rpm = angular_velocity * 60.0f / (2.0f * PI); // rad/s → RPM
    encoderData.cumulative_angle = cumulative_angle_deg; // degrees
    encoderData.rotations = total_rotations;
    encoderData.magnitude = mag_value;
    encoderData.agc_value = agc_value;
    encoderData.error_flags = error_flags;
    encoderData.timestamp = HAL_GetTick(); // 假设使用 HAL 库

    // ====== 6. 更新历史值 ======
    prev_raw_angle = raw_angle;
    old_mod_mech_position = mod_mech_position;
}

/**
 * @brief 获取编码器数据指针
 * @return EncoderData_t* 指向全局数据
 */
EncoderData_t *Encoder_GetData(void) {
    return &encoderData;
}
