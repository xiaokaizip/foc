//
// Created by SXF-Admin on 25-9-22.
//

#include "open_loop_controller.h"
#include "tim.h" // 假设 TIM1 定义在此头文件中

void open_loop_controller::updata(float period) {
    float Uq = -1.0f; // 测试负值
    float Ud = 0.0f;

    float speed = 7.0f; // 电角度速度 (rad/s)

    // ✅ 根据 Uq 符号决定 angle 增减
    // if (Uq >= 0.0f) {
    //     electric_angle += speed * period;
    // } else {
    //     electric_angle -= speed * period;
    // }
    electric_angle += speed * period;
    // ✅ 强制限制在 [0, 2π)
    electric_angle = fmodf(electric_angle, 2.0f * PI);
    // if (electric_angle < 0.0f) electric_angle += 2.0f * PI;

    // // Park 反变换
    // U_alpha = -Uq * sinf(electric_angle);
    // U_beta = Uq * cosf(electric_angle);
    //
    // // Clark 逆变换
    // Ua = U_alpha + voltage_power_supply / 2.0f;
    // Ub = (-0.5f * U_alpha + 0.8660254f * U_beta) + voltage_power_supply / 2.0f;
    // Uc = (-0.5f * U_alpha - 0.8660254f * U_beta) + voltage_power_supply / 2.0f;

    abc(electric_angle, 0, Uq, &Ua, &Ub, &Uc);

    setPWM(&Ua, &Ub, &Uc);
}

void open_loop_controller::setPWM(float *Ua, float *Ub, float *Uc) {
    *Ua += voltage_power_supply / 2;
    *Ub += voltage_power_supply / 2;
    *Uc += voltage_power_supply / 2;
    limit(Ua, 0.0f, voltage_limit);
    limit(Ub, 0.0f, voltage_limit);
    limit(Uc, 0.0f, voltage_limit);


    dc_a = *Ua / voltage_power_supply;
    dc_b = *Ub / voltage_power_supply;
    dc_c = *Uc / voltage_power_supply;

    limit(&dc_a, 0.0f, 1.0f);
    limit(&dc_b, 0.0f, 1.0f);
    limit(&dc_c, 0.0f, 1.0f);

    // 设置 PWM 寄存器
    TIM1->CCR1 = static_cast<int>((dc_a) * 4200.0f);
    TIM1->CCR2 = static_cast<int>((dc_b) * 4200.0f);
    TIM1->CCR3 = static_cast<int>((dc_c) * 4200.0f);
}

void open_loop_controller::abc(float theta, float d, float q, float *a, float *b, float *c) {
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *a = cf * d - sf * q;
    *b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
    *c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}
