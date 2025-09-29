//
// Created by SXF-Admin on 25-9-22.
//

#include "open_loop_controller.h"
#include "tim.h" // 假设 TIM1 定义在此头文件中

void open_loop_controller::updata(float electric_angle, float Ud, float Uq) {
    electric_angle = fmodf(electric_angle, 2.0f * PI);

    abc(electric_angle, Ud, Uq, &Ua, &Ub, &Uc);

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
    TIM1->CCR1 = static_cast<int>((1 - dc_a) * 8400.0f);
    TIM1->CCR2 = static_cast<int>((1 - dc_b) * 8400.0f);
    TIM1->CCR3 = static_cast<int>((1 - dc_c) * 8400.0f);
}

void open_loop_controller::abc(float theta, float d, float q, float *a, float *b, float *c) {
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *a = cf * d - sf * q;
    *b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
    *c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}
