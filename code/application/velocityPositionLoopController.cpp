//
// Created by SXF-Admin on 25-9-22.
//

#include "velocityPositionLoopController.h"
#include "math_ops.h"
#include "FastMath.h"

void velocityPositionLoopController::updata(float expect_velocity, float expect_position) {
    // 速度误差
    velocity_error = expect_velocity - encoderData.angular_velocity;
    velocity_error_sum += velocity_error;
    limit(&velocity_error_sum, velocity_error_sum_min, velocity_error_sum_max);
    //
    // // PI 控制器输出 Uq
    // Uq = velocity_error * velocity_kp + velocity_error_sum * velocity_ki;
    // limit(&Uq, Uq_min, Uq_max);

    position_error = expect_position - encoderData.cumulative_angle;
    // Uq = position_error * position_kp + velocity_error * velocity_kp;
    Uq = velocity_error * velocity_kp + velocity_error_sum * velocity_ki;
    limit(&Uq, -1, 1);
    // 设 d 轴电压为 0，q 轴电压为 Uq
    Ud = 0.0f;

    // 获取电角度（来自编码器）
    electric_angle = encoderData.electrical_angle - PI;

    abc(electric_angle, 0, Uq, &Ua, &Ub, &Uc);

    // 复用父类的 PWM 设置
    setPWM(&Ua, &Ub, &Uc);
}

// 可选：实现 dq0 变换用于电流采样等后续功能
void velocityPositionLoopController::dq0(float theta, float a, float b, float c, float *d, float *q) {
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - 0.5f * cf) * b + (-0.86602540378f * sf - 0.5f * cf) * c);
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - 0.5f * sf) * b - (0.86602540378f * cf - 0.5f * sf) * c);
}
