#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "foc.h"
#include "PositionSensor.h"
#include "PreferenceWriter.h"
#include "Config/user_config.h"

#define V_CAL 0.5f;


void order_phases(PositionSensor *ps, ControllerStruct *controller, PreferenceWriter *prefs);

void calibrate(PositionSensor *ps, ControllerStruct *controller, PreferenceWriter *prefs);
#endif
