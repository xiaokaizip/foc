// FOC.h
//
// Created by kade on 2025/9/24.
// Modified for PI control with anti-windup and encapsulation

#ifndef FOC_FOC_H
#define FOC_FOC_H

#include "currentCalibration.h"

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
    bool anti_windup; // 是否启用抗饱和
};

// PI 控制器计算函数（声明）
float pi_controller_calculate(pi_controller_t *pi);


class currentControl : public currentCalibration {
public:
    // ========== 构造与析构 ==========
    currentControl() = default;

    ~currentControl() = default;

    // ========== 控制使能 ==========
    void enableControl(bool en) { enable = en; }
    bool &isEnabled() { return enable; }

    // ========== 参考值设置接口 ==========
    void setTorqueReference(const float torque) {
        torque_ref = torque;
    }

    void setVelocityReference(const float vel) {
        velocity_ref = vel;
    }

    void setPositionReference(const float pos) {
        position_ref = pos;
    }

    // ========== 状态获取接口 ==========
    float &getTorque() { return torque; }
    float &getVelocity() { return velocity; }
    float &getPosition() { return position; }
    float &getIq() { return Iq; }
    float &getId() { return Id; }


    float &getPositionReference() { return position_ref; }
    float &getVelocityReference() { return velocity_ref; }
    float &getTorqueReference() { return torque_ref; }

    float &getCurrentA() { return currentA; }
    float &getCurrentB() { return currentB; }
    float &getCurrentC() { return currentC; }


    void getPWMOutputs(float &ua, float &ub, float &uc) const;

    // ========== 主控制循环接口 ==========
    void velocityPositionLoop(); // 外环：位置/速度控制
    void currentLoop(); // 内环：电流控制

private:
    // ========== 常量定义 ==========
    static constexpr float GAIN_CURRENT_SAMPLE = 3.3f / 4096.0f / 40.0f / 0.05f;
    static constexpr float MAX_TORQUE = 3.0f;
    static constexpr float MAX_VELOCITY = 3000.0f;
    static constexpr float DTC_MIN = 0.0f;
    static constexpr float DTC_MAX = 0.9f;

    // ========== 控制标志与参考值 ==========
    bool enable = false;

    float torque_ref = 0.0f;
    float velocity_ref = 0.0f;
    float position_ref = 0.0f;

    // ========== 状态反馈 ==========
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f; // 可由 Iq 估算

    // ========== 电流采样与变换 ==========
    float currentA = 0.0f;
    float currentB = 0.0f;
    float currentC = 0.0f;

    float Iq = 0.0f;
    float Id = 0.0f;

    float Iq_ref = 0.0f;
    float Id_ref = 0.0f;

    float Uq = 0.0f;
    float Ud = 0.0f;

    // ========== 电压与PWM输出 ==========
    float U_alpha = 0.0f;
    float U_beta = 0.0f;
    float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
    float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;

    float electric_angle = 0.0f;
    float theta = 0.0f; // 电角度

    // ========== PI 控制器（全部私有）==========
    pi_controller_t positionController = {
        5.0f, 0.0f, // kp, ki
        0.0f, 0.0f, // error, error_sum
        100.0f, -100.0f, // error_sum_max/min
        0.0f, // out
        100.0f, -100.0f, // out_max, out_min
        false // anti_windup
    };

    pi_controller_t velocityController = {
        0.005f, 0.0002f,
        0.0f, 0.0f,
        10000.0f, -10000.0f,
        0.0f,
        1.0f, -1.0f,
        true
    };

    pi_controller_t IqController = {
        5.0f, 0.02f,
        0.0f, 0.0f,
        10000.0f, -10000.0f,
        0.0f,
        2.0f, -2.0f,
        true
    };

    pi_controller_t IdController = {
        0.05f, 0.01f,
        0.0f, 0.0f,
        10000.0f, -10000.0f,
        0.0f,
        2.0f, -2.0f,
        true
    };

    // ========== 电源电压 ==========
    float voltage_power_supply = 24.0f;

    // ========== 内部方法 ==========
    void current_calculation();

    static void abc(float theta, float d, float q, float *a, float *b, float *c);

    static void dq0(float theta, float a, float b, float c, float *d, float *q);

    void setPWM(float *Ua, float *Ub, float *Uc);

    void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
};

#endif // FOC_FOC_H
