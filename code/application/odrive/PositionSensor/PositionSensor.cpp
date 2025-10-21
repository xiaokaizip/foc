
#include "PositionSensor.h"
#include "math_ops.h"
#include <string.h>
//#include "offset_lut.h"
//#include <math.h>

extern "C" {
#include "AS5047.h"
}

PositionSensorAM5047::PositionSensorAM5047(int CPR, float offset, int ppairs) {
    //_CPR = CPR;
    _CPR = CPR;
    _ppairs = ppairs;
    ElecOffset = offset;
    rotations = 0;
    readAngleCmd = 0xffff;
    MechOffset = offset;
    modPosition = 0;
    oldModPosition = 0;
    oldVel = 0;
    raw = 0;
}

void PositionSensorAM5047::Sample(float dt) {
    int angle = AS5047_ReadData(0, ANGLECOM_AS5047P_VOL_REG_ADD);;
    // Correct for nonlinearity with lookup table from calibration
    if (angle - old_counts > _CPR / 2) {
        rotations -= 1;
    } else if (angle - old_counts < -_CPR / 2) {
        rotations += 1;
    }

    old_counts = angle;
    oldModPosition = modPosition;
    modPosition = ((2.0f * PI * ((float) angle)) / (float) _CPR);
    position = (2.0f * PI * ((float) angle + (_CPR * rotations))) / (float) _CPR;
    MechPosition = position - MechOffset;
    float elec = ((2.0f * PI / (float) _CPR) * (float) ((_ppairs * angle) % _CPR)) + ElecOffset;
    if (elec < 0) elec += 2.0f * PI;
    else if (elec > 2.0f * PI) elec -= 2.0f * PI;
    ElecPosition = elec;

    float vel;
    //if(modPosition<.1f && oldModPosition>6.1f){

    if ((modPosition - oldModPosition) < -3.0f) {
        vel = (modPosition - oldModPosition + 2.0f * PI) / dt;
    }
    //else if(modPosition>6.1f && oldModPosition<0.1f){
    else if ((modPosition - oldModPosition) > 3.0f) {
        vel = (modPosition - oldModPosition - 2.0f * PI) / dt;
    } else {
        vel = (modPosition - oldModPosition) / dt;
    }

    int n = 40;
    float sum = vel;
    for (int i = 1; i < (n); i++) {
        velVec[n - i] = velVec[n - i - 1];
        sum += velVec[n - i];
    }
    velVec[0] = vel;
    MechVelocity = sum / ((float) n);
    ElecVelocity = MechVelocity * _ppairs;
    ElecVelocityFilt = 0.99f * ElecVelocityFilt + 0.01f * ElecVelocity;
}

int PositionSensorAM5047::GetRawPosition() {
    return raw;
}

float PositionSensorAM5047::GetMechPositionFixed() {
    return MechPosition + MechOffset;
}

float PositionSensorAM5047::GetMechPosition() {
    return MechPosition;
}

float PositionSensorAM5047::GetElecPosition() {
    return ElecPosition;
}

float PositionSensorAM5047::GetElecVelocity() {
    return ElecVelocity;
}

float PositionSensorAM5047::GetMechVelocity() {
    return MechVelocity;
}

void PositionSensorAM5047::ZeroPosition() {
    rotations = 0;
    MechOffset = 0;
    Sample(.00025f);
    MechOffset = GetMechPosition();
}

void PositionSensorAM5047::SetElecOffset(float offset) {
    ElecOffset = offset;
}

void PositionSensorAM5047::SetMechOffset(float offset) {
    MechOffset = offset;
}

int PositionSensorAM5047::GetCPR() {
    return _CPR;
}


void PositionSensorAM5047::WriteLUT(int new_lut[128]) {
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
}

