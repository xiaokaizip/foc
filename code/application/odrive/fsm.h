//
// Created by SXF-Admin on 25-10-16.
//

#ifndef FSM_H
#define FSM_H

#include "main.h"
#include "foc.h"

#define REST_MODE 0
#define CALIBRATION_MODE 1
#define MOTOR_MODE 2
#define SETUP_MODE 4
#define ENCODER_MODE 5

void fsm();
#endif //FSM_H
