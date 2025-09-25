//
// Created by SXF-Admin on 25-9-22.
//

#ifndef VELOCITYPOSITIONLOOPCONTROLLER_H
#define VELOCITYPOSITIONLOOPCONTROLLER_H

#include "open_loop_controller.h"


class velocityPositionLoopController : public open_loop_controller {
public:
    enum pos_vel_mode_e {
        pos_vel,
        vel
    };
    pos_vel_mode_e pos_vel_mode = vel;

    // 更新速度环和位置环（目前只实现了速度环）
    void updata(float expect_velocity, float expect_position, float period, pos_vel_mode_e mode);

    float velocity_error = 0.0f;
    float velocity_error_sum = 0.0f;
    float electrical_angle_offset = 0.0f;

    float position_error = 0.0f;
    float position_error_sum = 0.0f;

private:
    // PID 参数
    float velocity_kp = 0.01f; // 示例值，可外部设置
    float velocity_ki = 0.0005;
    float velocity_error_sum_max = 10000.0f;
    float velocity_error_sum_min = -10000.0f;
    float Uq_max = 2.0f;
    float Uq_min = -2.0f;
    float Uq = 0.0f, Ud = 0.0f;

    float position = 0;
    float position_kp = 1.0f;
    float position2vel = 0;

    // 工具函数：DQ0 正变换（Clark + Park）
    void dq0(float theta, float a, float b, float c, float *d, float *q);
};

#endif // VELOCITYPOSITIONLOOPCONTROLLER_H
