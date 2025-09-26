// FOC.cpp
#include "currentCalibration.h"
#include "currentControl.h"
#include <algorithm> // for std::clamp
#include <position_sensor.h>
#include "tim.h"
// PI 控制器计算函数（带抗饱和）
float pi_controller_calculate(pi_controller_t *pi) {
    // 比例项

    // 积分项（先更新）
    pi->error_sum += pi->error;

    // 【关键】积分限幅（Anti-windup 第一步）
    if (pi->error_sum > pi->error_sum_max) {
        pi->error_sum = pi->error_sum_max;
    } else if (pi->error_sum < pi->error_sum_min) {
        pi->error_sum = pi->error_sum_min;
    }

    // 总输出
    pi->out = pi->kp * pi->error + pi->error_sum * pi->ki;

    // 输出限幅
    if (pi->out > pi->out_max) {
        pi->out = pi->out_max;
    } else if (pi->out < pi->out_min) {
        pi->out = pi->out_min;
    }

    // // 【增强抗饱和】如果输出已饱和，且误差方向会导致积分继续增长，则停止积分更新
    // // 这是“Conditional Integration”或“Back-Calculation”思想的简化版
    // if (pi->anti_windup) {
    //     bool is_saturated = false;
    //     if ((pi->out >= pi->out_max && pi->error > 0) ||
    //         (pi->out <= pi->out_min && pi->error < 0)) {
    //         is_saturated = true;
    //     }
    //
    //     if (is_saturated) {
    //         // 停止积分累加（防止 windup）
    //         pi->error_sum -= pi->ki * pi->error; // 撤销本次积分更新
    //     }
    // }

    return pi->out;
}

// 更新函数（主控制循环调用）
void currentControl::velocityPositionLoop() {
    // （来自编码器）
    theta = encoderData.electrical_angle; //来自编码器的电角度
    velocity = encoderData.angular_velocity; //来自编码器的角速度
    position = encoderData.mechanical_angle; //来自编码器的速度，使用编码器的角度的话，确定好方向
    float pos2Vel = 0;

    positionController.error = -position_ref + position;
    pos2Vel = pi_controller_calculate(&positionController);

    limit(&pos2Vel, -velocity_ref, velocity_ref);

    // 5. 外环：位置或速度环（示例为速度环）
    velocityController.error = pos2Vel - velocity;
    pi_controller_calculate(&velocityController);
    Iq_ref = velocityController.out; // 作为电流环 q轴参考值
    limit(&Iq_ref, -torque_ref, torque_ref);

    // // 6. 内环：Id/Iq 电流环
    // IqController.error = Iq_ref - Iq;
    // pi_controller_calculate(&IqController);
    //
    // IdController.error = Id_ref - Id;
    // pi_controller_calculate(&IdController);

    // 7. 输出：Vq = IqController.out, Vd = IdController.out
    // 后续可进行逆 Park 变换生成 SVPWM 指令
}

void currentControl::currentLoop() {
    // 2. 电流采样与计算
    current_calculation();

    dq0(theta, currentA, currentB, currentC, &Id, &Iq);

    IqController.error = Iq_ref - Iq;
    Uq = pi_controller_calculate(&IqController);

    IdController.error = 0 - Id;
    Ud = pi_controller_calculate(&IdController);


    abc(theta, Ud, Uq, &Ua, &Ub, &Uc);
    setPWM(&Ua, &Ub, &Uc);
}

void currentControl::current_calculation() {
    // 假设 raw_current_A/B 是从 ADC 读取的原始值
    // 减去零点偏移（已校准）
    currentA = (static_cast<float>(current[0]) - current_offset[0]) * GAIN_CURRENT_SAMPLE;
    currentB = (static_cast<float>(current[1]) - current_offset[1]) * GAIN_CURRENT_SAMPLE;
    currentC = -(currentA + currentB); // KCL
}


void currentControl::abc(float theta, float d, float q, float *a, float *b, float *c) {
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *a = cf * d - sf * q;
    *b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
    *c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}

// 可选：实现 dq0 变换用于电流采样等后续功能
void currentControl::dq0(float theta, float a, float b, float c, float *d, float *q) {
    const float cf = FastCos(theta);
    const float sf = FastSin(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - 0.5f * cf) * b + (-0.86602540378f * sf - 0.5f * cf) * c);
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - 0.5f * sf) * b - (0.86602540378f * cf - 0.5f * sf) * c);
}

void currentControl::setPWM(float *Ua, float *Ub, float *Uc) {
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
