//
// Created by SXF-Admin on 26-1-26.
//

#ifndef CONFIG_H
#define CONFIG_H
#include "parameters_conversion.h"

#define ADC_VOLTAGE_SCALING_FACTOR (13.0f/4095.0f*3.3f)
#define ADC_CURRENT_SCALING_FACTOR (1.0f/0.05f/6.0f/4095.0f*3.3f)


#define ADC1_VOLTAGE_U_INDEX 3
#define ADC1_VOLTAGE_V_INDEX 2
#define ADC1_VOLTAGE_W_INDEX 1
#define ADC1_CURRENT_V_INDEX 4

#define ADC_CURRENT_U_INDEX 0xFF

#define ADC2_VOLTAGE_BUS_INDEX 1


#define ADC2_CURRENT_W_INDEX 2

//如果开启DEBUG，则使用JustFloat的格式进行数据传输，否则使用带有CRC校验的函数。
#define DEBUG 1

#define DTC_MAX 0.94f          // Max phase duty cycle
#define DTC_MIN 0.0f          // Min phase duty cycle
#define PWM_ARR ((PWM_PERIOD_CYCLES/2))          /// timer autoreload value


#define PHASE_ORDER 1

#endif //CONFIG_H
