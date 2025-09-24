//
// Created by SXF-Admin on 25-9-22.
//

#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <iostream>

// 假设 EncoderData_t 和 encoderData 在全局作用域中定义
extern "C" {
#include "math_ops.h"
#include "FastMath.h"
#include "position_sensor.h"
}

extern EncoderData_t encoderData;

class open_loop_controller {
public:
    open_loop_controller() = default;

    virtual ~open_loop_controller() = default;

    // 开环控制更新
    virtual void updata(float period);

    // 提供对 PWM 占空比的访问
    float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;

protected:
    // 可被子类访问的公共资源
    float voltage_limit = 26.0f; // 可设默认值或运行时配置
    float voltage_power_supply = 26.0f; // 假设电源电压为 26V

    float U_alpha = 0.0f, U_beta = 0.0f;
    float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
    float electric_angle = 0.0f;

    // PWM 设置方法（子类也可调用）
    void setPWM(float *Ua, float *Ub, float *Uc);

    // DQ 反变换（Clark + Park 逆变换），可作为工具函数提取
    void abc(float theta, float d, float q, float *a, float *b, float *c);
};

#endif // OPEN_LOOP_CONTROLLER_H
