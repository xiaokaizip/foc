#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "drv8301.h"
#include "tim.h"
#include "adc.h""
#include "application.h"
#include "open_loop_controller.h"
#include "velocityPositionLoopController.h"
#include "position_sensor.h"

extern "C" {
#include "lwprintf/lwprintf.h"
#include "vofa_plus.h"


void SystemClock_Config(void);
}

int uart_out(int ch, lwprintf_t *lwp);

open_loop_controller open_loop;
velocityPositionLoopController velocityPositionLoopController;

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI3_Init();

    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM14_Init();

    MX_ADC1_Init();

    lwprintf_init(uart_out); // 默认实例
    lwprintf_printf("init foc\n\r");
    Drv8301 m0_gate_driver{
        &hspi3,
        M0_nCS_GPIO_Port, M0_nCS_Pin,
        EN_GATE_GPIO_Port, EN_GATE_Pin,
        nfault_GPIO_Port, nfault_Pin
    };

    float gain = 40;
    m0_gate_driver.config(40, &gain);
    while (m0_gate_driver.init() != true) {
        lwprintf_printf("drv8301 init successfully\n\r");
        HAL_Delay(1000);
    }

    HAL_TIM_Base_Start_IT(&htim14);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);


    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC); // 启动注入采样中断
    HAL_ADCEx_InjectedStart(&hadc1); // 开启注入采样


    while (1) {
        // m0_gate_driver.get_error();
        send_to_vofa();
        HAL_Delay(1);
    }
}

int uart_out(int ch, lwprintf_t *lwp) {
    if (ch != '\0') {
        HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 10);
    }
    return ch;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {
        // 检查是否是TIM3中断

        Encoder_Update(0, 0.001f); // 更新编码器数据(使用devidx=0)
        // open_loop.updata(0.001f);
        velocityPositionLoopController.updata(10, 1);
    }
}


unsigned int gADC_IN10, gADC_IN11;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        gADC_IN10 = hadc->Instance->JDR1; // Injected Rank1
        gADC_IN11 = hadc->Instance->JDR2; // Injected Rank2
    }
}


