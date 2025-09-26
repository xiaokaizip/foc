#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "drv8301.h"
#include "tim.h"
#include "adc.h""
#include "dma.h"
#include "application.h"
#include "loop/open_loop_controller.h"
#include "loop/velocityPositionLoopController.h"
#include "position_sensor.h"
#include "loop/currentControl.h"
#include "serial_modbus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "delay.h"
#include "currentCalibration.h"

extern "C" {
#include "lwprintf/lwprintf.h"
#include "vofa_plus.h"


void SystemClock_Config(void);

void MX_FREERTOS_Init(void);
}

int uart_out(int ch, lwprintf_t *lwp);

unsigned char uart_rxdata[10];
// open_loop_controller open_loop;
// velocityPositionLoopController velocityPositionLoopController;
currentControl foc;
SerialLogger_t g_serial_logger;
Drv8301 m0_gate_driver{
    &hspi3,
    M0_nCS_GPIO_Port, M0_nCS_Pin,
    EN_GATE_GPIO_Port, EN_GATE_Pin,
    nfault_GPIO_Port, nfault_Pin
};
unsigned int gADC_IN10, gADC_IN11;


void packet_uart2vofa() {
    SerialLogger_AddDataToChannel(&g_serial_logger, 0, &encoderData.mechanical_angle);
    SerialLogger_AddDataToChannel(&g_serial_logger, 1, &encoderData.electrical_angle);
    SerialLogger_AddDataToChannel(&g_serial_logger, 2, &encoderData.angular_velocity);
    SerialLogger_AddDataToChannel(&g_serial_logger, 3, &encoderData.rpm);
    SerialLogger_AddDataToChannel(&g_serial_logger, 4, &encoderData.cumulative_angle);
    SerialLogger_AddDataToChannel(&g_serial_logger, 5, &foc.currentA);
    SerialLogger_AddDataToChannel(&g_serial_logger, 6, &foc.currentB);
    SerialLogger_AddDataToChannel(&g_serial_logger, 7, &foc.currentC);
    SerialLogger_AddDataToChannel(&g_serial_logger, 8, &foc.Id);
    SerialLogger_AddDataToChannel(&g_serial_logger, 9, &foc.Iq);
    SerialLogger_AddDataToChannel(&g_serial_logger, 10, &foc.position_ref);
    SerialLogger_AddDataToChannel(&g_serial_logger, 11, &foc.velocity_ref);
    SerialLogger_AddDataToChannel(&g_serial_logger, 12, &foc.torque_ref);
    SerialLogger_AddDataToChannel(&g_serial_logger, 13, (float *) &foc.enable);
    SerialLogger_AddDataToChannel(&g_serial_logger, 14, &foc.velocityController.error);
    SerialLogger_AddDataToChannel(&g_serial_logger, 15, &foc.velocityController.error_sum);
    SerialLogger_AddDataToChannel(&g_serial_logger, 16, &foc.positionController.error);
    SerialLogger_AddDataToChannel(&g_serial_logger, 17, &foc.positionController.error_sum);
    SerialLogger_AddDataToChannel(&g_serial_logger, 18, &foc.Iq);
    SerialLogger_AddDataToChannel(&g_serial_logger, 19, &foc.Iq_ref);
    SerialLogger_AddDataToChannel(&g_serial_logger, 20, &foc.Uq);
    SerialLogger_AddDataToChannel(&g_serial_logger, 21, &foc.Id);
}

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

    SerialLogger_Init(&g_serial_logger, &huart4);
    packet_uart2vofa();

    SerialLogger_StartReceive(&g_serial_logger);


    while (1) {
        // m0_gate_driver.fault_status = m0_gate_driver.get_error();
        if (g_serial_logger.is_update == true) {
            foc.enable = g_serial_logger.command.enabled;
            foc.position_ref = g_serial_logger.command.target_position;
            foc.torque_ref = g_serial_logger.command.target_torque;
            foc.velocity_ref = g_serial_logger.command.target_velocity;
            g_serial_logger.is_update = false;
        }
        SerialLogger_SendToVofa(&g_serial_logger);
        HAL_Delay(1);
    }
}

int uart_out(int ch, lwprintf_t *lwp) {
    if (ch != '\0') {
        HAL_UART_Transmit(&huart4, (uint8_t *) &ch, 1, 10);
    }
    return ch;
}

unsigned char flag = 0;
unsigned short count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {
        foc.calibrate(0.001f);
        // open_loop.updata(0.001f);
        Encoder_Update(0, 0.001f); // 更新编码器数据(使用devidx=0)

        if (foc.enable == true) {
            foc.velocityPositionLoop();;
        } else {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
        }
    }
    if (htim->Instance == TIM13) {
        HAL_IncTick();
    }
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        foc.current[1] = hadc->Instance->JDR2; // Injected Rank2
        foc.current[0] = hadc->Instance->JDR1;
        if (foc.enable == true) {
            foc.currentLoop();
        } else {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
        }
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart->Instance == UART4) {
//     }
// }

