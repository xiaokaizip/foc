//
// Created by SXF-Admin on 25-10-14.
//
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "drv8301.h"
#include "tim.h"
#include "adc.h"
#include "dma.h"

extern "C" {
#include "lwprintf/lwprintf.h"
#include "../vofa_plus.h"


void SystemClock_Config(void);

void MX_FREERTOS_Init(void);
}

int uart_out(int ch, lwprintf_t *lwp);



int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI3_Init();
    MX_DMA_Init();
    MX_UART4_Init();
    MX_TIM1_Init();
    MX_TIM14_Init();
    MX_TIM12_Init();
    MX_ADC1_Init();
    delay_us_init(&htim12);
    lwprintf_init(uart_out); // 默认实例
    lwprintf_printf("init foc\n\r");


    float gain = 40;
    m0_gate_driver.config(40, &gain);
    while (m0_gate_driver.init() != true) {
        lwprintf_printf("drv8301 init successfully\n\r");
        HAL_Delay(1000);
    }
    HAL_Delay(1000);

    HAL_TIM_Base_Start_IT(&htim14);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC); // 启动注入采样中断
    HAL_ADCEx_InjectedStart(&hadc1); // 开启注入采样

    while (1) {
        HAL_Delay(1);
    }
}

int uart_out(int ch, lwprintf_t *lwp) {
    if (ch != '\0') {
        HAL_UART_Transmit(&huart4, reinterpret_cast<uint8_t *>(&ch), 1, 10);
    }
    return ch;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {

    }
    if (htim->Instance == TIM13) {
    }
}

int curretnA = 0;
int curretnB = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        curretnA = hadc->Instance->JDR2; // Injected Rank2
        curretnB = hadc->Instance->JDR1;
    }
}


