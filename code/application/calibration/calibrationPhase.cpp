/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#include "calibrationPhase.h"
#include "math_ops.h"
#include "loop/open_loop_controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#define _3PI_BY_2 (3.1415926f*3.0f/2.0f)

open_loop_controller open_loop_controller;
calibration_t calibration;

void calibrate(const bool is_calibrated) {
    if (is_calibrated == false) return;
    for (int i = 0; i < 1000; i++) {
        float angle = _3PI_BY_2 + 2 * PI * static_cast<float>(i) / 1000.0f;
        open_loop_controller.updata(angle, 0.0f, 0.5f);
        vTaskDelay(2);
    }
    float mid_angle = encoderData.cumulative_angle;
    for (int i = 1000; i > 0; i--) {
        float angle = _3PI_BY_2 + 2 * PI * static_cast<float>(i) / 1000.0f;
        open_loop_controller.updata(angle, 0.0f, 0.5f);
        vTaskDelay(2);
    }

    float end_angle = encoderData.cumulative_angle;
    open_loop_controller.updata(0, 0.0f, 0.0f);

    vTaskDelay(200);

    if (mid_angle < end_angle) {
        calibration.direction = -1;
    } else if (mid_angle > end_angle) {
        calibration.direction = 1;
    }

    float moved = fabsf(mid_angle - end_angle);
    uint8_t pp_check_result = !(fabsf(moved * 0.0f - 2.0f * 3.1415926f) > 0.5f);
    // 0.5f is arbitrary number it can be lower or higher!

    if (pp_check_result == false) {
        calibration.pp = static_cast<int>(2.0f * 3.1415926f / moved);
    }

    open_loop_controller.updata(_3PI_BY_2, 0.0f, 1.0f);
    vTaskDelay(1000);
    calibration.offset = encoderData.mechanical_angle;
    calibration.zero_electric_angle =
            fmodf(static_cast<float>(calibration.direction) * 7 * calibration.offset, 2 * PI);

    open_loop_controller.updata(0, 0.0f, 0.0f);
    vTaskDelay(200);
    vTaskDelay(200);
}
