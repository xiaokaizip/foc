//
// Created by SXF-Admin on 26-1-14.
//

#include "main.h"

#include <filesystem>

#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "lwprintf/lwprintf.h"
#include "config.h"
#include "SerialCommunication.h"
#include "dma.h"
#include "foc.h"
#include "SMC.h"
#include "stspin32g4.h"
#include "spi.h"
#include "encoder.h"

extern "C" {
extern void SystemClock_Config(void);
}

static int
lwprintf_my_out_func(int ch, lwprintf_t *p);

#define APP_BASE_ADDR    (0x08008000U)

unsigned int count = 0;

Controller controller;
EncoderMT6825 encoder;


int main(void) {
    /* USER CODE BEGIN 1 */
    // SCB->VTOR = APP_BASE_ADDR;
    // __enable_irq();

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();

    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_OPAMP2_Init();
    MX_OPAMP3_Init();
    MX_TIM1_Init();
    MX_TIM17_Init();
    MX_USART1_UART_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(10);

    // HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(adc_value), 5);
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start_IT(&htim17);

    HAL_ADCEx_InjectedStart_IT(&hadc2); // 开启注入采样
    HAL_ADCEx_InjectedStart_IT(&hadc1); // 开启注入采样

    lwprintf_init(lwprintf_my_out_func);

    /* Print formatted data */
    /* USER CODE END 2 */
    initProtocol(); // 上报时间单位

    SerialCommunication::Init();

    STSPIN32G4_HandleTypeDef gatedriver;

    STSPIN32G4_init(&gatedriver);
    STSPIN32G4_reset(&gatedriver);
    STSPIN32G4_confVCC confVCC;
    confVCC.voltage = STSPIN32G4_confVCC::_8V;
    confVCC.useNFAULT = true;
    confVCC.useREADY = false;
    STSPIN32G4_setVCC(&gatedriver, confVCC);
    STSPIN32G4_confVDSP confVDSP;
    confVDSP.deglitchTime = STSPIN32G4_confVDSP::_4us;
    confVDSP.useNFAULT = true;
    STSPIN32G4_setVDSP(&gatedriver, confVDSP);
    STSPIN32G4_clearFaults(&gatedriver);


    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // lwprintf_printf("hello\n");

        // SerialCommunication::printFloatsWithPacketType(0x03, 1.0f, 2.0f);
        // SerialCommunication::printFloatsWithConfig(SIMPLE_PROTOCOL, controller.foc.dtc_u, controller.foc.dtc_v,
        //                                            controller.foc.dtc_w, controller.foc.theta_elec, 4, 4); // 类似 print

//        SerialCommunication::printFloatsWithConfig(SIMPLE_PROTOCOL, controller.foc.i_u, controller.foc.i_v,
//                                                   controller.foc.i_w, controller.foc.theta_elec, controller.foc.v_bus,
//                                                   controller.foc.real_voltage_u,
//                                                   controller.smc_param.i_u, controller.smc_param.es_u,
//                                                   controller.foc.i_q,
//                                                   controller.smc_param.theta, controller.foc.i_alpha,
//                                                   controller.foc.i_beta); // 类似 print
        SerialCommunication::printFloatsWithConfig(SIMPLE_PROTOCOL, encoder.phase_vel()); // 类似 print

        if (SerialCommunication::parseRefParamFrame()) {
            controller.foc.v_des = SerialCommunication::refParam.refSpeed;
            // lwprintf_printf("%f,%f,%f\n", SerialCommunication::refParam.refSpeed,
            //                 SerialCommunication::refParam.refPosition, SerialCommunication::refParam.refCurrent);
            // 成功更新 refParam
            // 可设置标志或直接使用
        }

        HAL_Delay(1);
        // count = 0;
        sendDataPeriodically();

        // read_mt6825_position();

        // //电流校准
        // static uint32_t adc_buf[2];
        // static uint16_t data_num = 0;
        // if (controller.foc.I_Calibration_flag == 0) {
        //     data_num++;
        //     adc_buf[0] = adc_buf[0] + ADC1->JDR4;
        //     adc_buf[1] = adc_buf[1] + ADC2->JDR2;
        //     if (data_num == 40) {
        //         controller.foc.I_Calibration_flag = 1;
        //         controller.foc.I_v_offset = adc_buf[0] / 40;
        //         controller.foc.I_w_offset = adc_buf[1] / 40;
        //     }
        // } else {
        //     if (controller.foc.zero_Calibration_flag == 0) {
        //         controller.foc.v_des = 0;
        //         controller.foc.v_d = 2.0f;
        //         controller.foc.v_q = 0.0f;
        //         HAL_Delay(10);
        //         controller.foc.zero_Calibration_flag = 1;
        //     }
        //     controller.foc.v_des = 330;
        //     controller.foc.v_d = 0.0f;
        //     controller.foc.v_q = 2.0f;
        // }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM17) {
        encoder.update(0.001f);

        if (controller.foc.v_des == 0) {
            TIM1->CCR3 = (PWM_ARR >> 1); // Write duty cycles
            TIM1->CCR2 = (PWM_ARR >> 1);
            TIM1->CCR1 = (PWM_ARR >> 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        }
    }
}

float val[6];
float v_bus, voltage_u, voltage_v, voltage_w, current_u, current_v, current_w;

/*
 *工作模式：配置为 双 ADC 注入同步模式 (ADC_DUALMODE_INJECSIMULT)。在此模式下，ADC1 作为主设备（Master），ADC2 作为从设备（Slave），两者通过硬件同步，确保注入通道的转换在同一时刻启动。
 *触发机制：采用 硬件触发 方式，由 TIM1 的 TRGO 信号（上升沿） 统一触发注入通道的转换，以保证采样时刻与 PWM 控制周期严格同步。
 *ADC1 和 ADC2 共用 ADC1_2_IRQn 中断通道，并已使能。
 *初始化：在初始化的时候，需要先初始化SLAVE（ADC2）的中断，再初始化MASTER（ADC1）的中断。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        if (controller.foc.I_Calibration_flag == 1) {
            controller.foc.real_voltage_u = ADC1->JDR1 * ADC_VOLTAGE_SCALING_FACTOR;
            controller.foc.real_voltage_v = ADC1->JDR2 * ADC_VOLTAGE_SCALING_FACTOR;
            controller.foc.real_voltage_w = ADC1->JDR3 * ADC_VOLTAGE_SCALING_FACTOR;

            controller.foc.i_v = static_cast<float>(static_cast<int>(ADC1->JDR4) - controller.foc.I_v_offset) *
                                 ADC_CURRENT_SCALING_FACTOR;

            controller.foc.v_bus = ADC2->JDR1 * ADC_VOLTAGE_SCALING_FACTOR;

            controller.foc.i_w = static_cast<float>(static_cast<int>(ADC2->JDR2) - controller.foc.I_w_offset) *
                                 ADC_CURRENT_SCALING_FACTOR;
            controller.foc.i_u = -(controller.foc.i_v + controller.foc.i_w);


            controller.open_loop(1 / 16000.0f);
            controller.SmcObserver();
        }
    }
}

static int
lwprintf_my_out_func(int ch, lwprintf_t *p) {
    uint8_t c = (uint8_t) ch;

    /* Don't print zero */
    if (c == '\0') {
        return ch;
    }
    HAL_UART_Transmit(&huart1, &c, 1, 100);
    return ch;
}
