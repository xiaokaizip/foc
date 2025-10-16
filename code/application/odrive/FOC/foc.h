#ifndef FOC_H
#define FOC_H

#include "../structs.h"
#include "PositionSensor/PositionSensor.h"
#include "Config/hw_config.h"
#include "math.h"
#include "math_ops.h"
#include "Config/motor_config.h"
#include "Config/current_controller_config.h"
#include "FastMath.h"
#include "Config/user_config.h"

void abc(float theta, float d, float q, float *a, float *b, float *c);

void dq0(float theta, float a, float b, float c, float *d, float *q);

void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);

void zero_current(int *offset_1, int *offset_2, const int adc1_offset, const int adc2_offset);

void reset_foc(ControllerStruct *controller);

// void reset_observer(ObserverStruct * observer);

void init_controller_params(ControllerStruct *controller);

void commutate(ControllerStruct *controller, float theta);

void torque_control(ControllerStruct *controller);

void limit_current_ref(ControllerStruct *controller);

void linearize_dtc(float *dtc);
#endif
