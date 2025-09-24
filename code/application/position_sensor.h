// position_sensor.h
#ifndef POSITION_SENSOR_H_
#define POSITION_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 数据结构
typedef struct {
    float mechanical_angle;
    float electrical_angle;
    float angular_velocity;
    float rpm;
    int rotations;
    float cumulative_angle;
    uint16_t raw_value;
    uint16_t magnitude;
    uint8_t agc_value;
    uint16_t error_flags;
    uint32_t timestamp;
} EncoderData_t;

// 函数声明
void Encoder_Update(int devidx, float dt);

EncoderData_t *Encoder_GetData(void);

#ifdef __cplusplus
}
#endif

// 全局变量声明
extern EncoderData_t encoderData;

#endif // POSITION_SENSOR_H_
