/**
 * There is the function of delay in this file.
 */

#include "main.h"

#include "delay.h"

static TIM_HandleTypeDef *delay_tim = NULL;

void delay_us_init(TIM_HandleTypeDef *htim) {
    delay_tim = htim;
    HAL_TIM_Base_Start(delay_tim);
}

void delayus(uint32_t us) {
    __HAL_TIM_SET_COUNTER(delay_tim, 0);
    while (__HAL_TIM_GET_COUNTER(delay_tim) < us) {
        // 空循环
    }
}

