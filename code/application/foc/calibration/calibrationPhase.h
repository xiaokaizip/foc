/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "../position_sensor.h"
#include<iostream>

struct calibration_t {
    float zero_electric_angle;
    int direction;
    int pp;
    float offset;
};

void calibrate(const bool is_calibrated);

extern calibration_t calibration;

#endif /* INC_CALIBRATION_H_ */
