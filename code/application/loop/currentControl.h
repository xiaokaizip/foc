// FOC.h
//
// Created by kade on 2025/9/24.
// Modified for PI control with anti-windup

#ifndef FOC_FOC_H
#define FOC_FOC_H

#include "currentCalibration.h"
#include "open_loop_controller.h"
#include "velocityPositionLoopController.h"

#define GAIN_CURRENT_SAMPLE (3.3f/4096.0f/40.0f/0.05f)

// PI 控制器结构体（带抗饱和）
struct pi_controller_t {
    float kp; // 比例增益
    float ki; // 积分增益
    float error; // 当前误差
    float error_sum; // 积分项（累积误差）
    float error_sum_max; // 积分上限（防饱和）
    float error_sum_min; // 积分下限
    float out; // 输出值
    float out_max; // 输出上限
    float out_min; // 输出下限

    // 【新增】用于抗饱和的标志位或前馈项（可选）
    bool anti_windup; // 是否启用抗饱和
};

// PI 控制器计算函数（声明）
float pi_controller_calculate(pi_controller_t *pi);

class currentControl : public currentCalibration {
public:
    float currentA = 0.0f;
    float currentB = 0.0f;
    float currentC = 0.0f;

    float Iq = 0.0f; // q轴电流（转矩分量）
    float Id = 0.0f; // d轴电流（励磁分量）

    float Uq = 0;
    float Ud = 0.0f;

    bool enable = false;
    float velocity_ref = 0.0f;
    float position_ref = 0.0f;
    float torque_ref = 0.0f;

    float Iq_ref = 0.0f; // q轴电流参考值（来自速度环）
    float Id_ref = 0.0f; // d轴电流参考值（通常为0）
    float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;
    float position = 0;
    float velocity = 0;
    float torque = 0;

    void velocityPositionLoop();

    void currentLoop();

    // PI 控制器实例
    pi_controller_t positionController = {
        5.0f, 0.0f, // kp, ki
        0.0f, 0.0f, // error, error_sum
        100.0f, -100.0f, // error_sum_max/min
        0.0f, // out
        100.0f, -100.0f, // out_max, out_min
        false // anti_windup (位置环通常不需要)
    };

    pi_controller_t velocityController = {
        0.001f, 0.00002f,
        0.0f, 0.0f,
        10000.0f, -10000.0f,
        0.0f,
        1.0f, -1.0f, // 限制输出为 ±3000 rpm 等效
        true // 启用抗饱和
    };

    pi_controller_t IqController = {
        5.0f, 0.02f, // 可根据系统调整
        0.0f, 0.0f,
        100.0f, -100.0f,
        0.0f,
        2.0f, -2.0f, // 电压输出限幅（Vq）
        true
    };

    pi_controller_t IdController = {
        0.05f, 0.01f,
        0.0f, 0.0f,
        10000.0f, -10000.0f,
        0.0f,
        2.0f, -2, // Vd
        true
    };

private:
    float voltage_limit = 24.0f; // 可设默认值或运行时配置
    float voltage_power_supply = 24.0f; // 假设电源电压为 26V

    float U_alpha = 0.0f, U_beta = 0.0f;
    float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
    float electric_angle = 0.0f;

    // 电流采样与坐标变换
    void current_calculation();

    static void abc(float theta, float d, float q, float *a, float *b, float *c);

    static void dq0(float theta, float a, float b, float c, float *d, float *q);

    void setPWM(float *Ua, float *Ub, float *Uc);

    void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);

    // Clark 和 Park 变换（可后续添加）
    float alpha = 0.0f, beta = 0.0f;
    float theta = 0.0f; // 电角度（来自编码器）
};

#endif // FOC_FOC_H
