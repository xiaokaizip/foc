

#ifndef MOTOR_CONTROL_CODE_DELAY_H
#define MOTOR_CONTROL_CODE_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

void delay_us_init(TIM_HandleTypeDef *htim);

void delayus(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif //MOTOR_CONTROL_CODE_DELAY_H
