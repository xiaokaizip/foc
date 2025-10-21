//
// Created by SXF-Admin on 25-10-14.
//

#include "main.h"
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "drv8301.h"
#include "tim.h"
#include "adc.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "delay.h"
#include "serial_modbus.h"
#include "PositionSensor/PositionSensor.h"
#include "Calibration/calibration.h"
#include "comnunication/serial_common.h"
#include "fsm.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

extern "C" {
#include "lwprintf/lwprintf.h"


void SystemClock_Config(void);

void MX_FREERTOS_Init(void);
}

int uart_out(int ch, lwprintf_t *lwp);

Drv8301 m0_gate_driver{
    &hspi3,
    M0_nCS_GPIO_Port, M0_nCS_Pin,
    EN_GATE_GPIO_Port, EN_GATE_Pin,
    nfault_GPIO_Port, nfault_Pin
};
PositionSensorAM5047 ps(16384,E_OFFSET, 7);

ControllerStruct controller;
PreferenceWriter prefs(6);

float __float_reg[64]; // Floats stored in flash
int __int_reg[256];


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
    MX_USB_DEVICE_Init();

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


    HAL_Delay(1000);
    zero_current(&controller.adc1_offset, &controller.adc2_offset, controller.adc1_raw,
                 controller.adc2_raw); // Measure current sensor zero-offset

    prefs.load(); // Read flash
    I_MAX = 1.0f;
    if (isnan(E_OFFSET)) { E_OFFSET = 0.0f; }
    if (isnan(M_OFFSET)) { M_OFFSET = 0.0f; }
    if (isnan(I_BW) || I_BW == -1) { I_BW = 1000; }
    if (isnan(I_MAX) || I_MAX == -1) { I_MAX = 1; }
    if (isnan(I_FW_MAX) || I_FW_MAX == -1) { I_FW_MAX = 0; }
    if (isnan(CAN_ID) || CAN_ID == -1) { CAN_ID = 1; }
    if (isnan(CAN_MASTER) || CAN_MASTER == -1) { CAN_MASTER = 0; }
    if (isnan(CAN_TIMEOUT) || CAN_TIMEOUT == -1) { CAN_TIMEOUT = 0; }
    ps.SetElecOffset(E_OFFSET); // Set position sensor offset
    ps.SetMechOffset(M_OFFSET);
    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));

    ps.WriteLUT(lut); // Set potision sensor nonlinearity lookup table
    init_controller_params(&controller);

    serial_init();

    controller.p_des = 10.0f;
    controller.v_des = 20.0f;
    controller.kp = 0.04f;
    controller.kd = 0.005f;
    controller.t_ff = 0.001f;
    controller.v_bus = 24.0f;

    char str[48] = "hello world\n";
    while (1) {
        // CDC_Transmit_FS((uint8_t *) str, strlen(str));
        HAL_Delay(1);
        serial_process();
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
        ps.Sample(0.001f);
        controller.theta_elec = ps.GetElecPosition();
        controller.theta_mech = (1.0f / GR) * ps.GetMechPosition();
        controller.dtheta_mech = (1.0f / GR) * ps.GetMechVelocity();
        controller.dtheta_elec = ps.GetElecVelocity();
        fsm();
    }
    if (htim->Instance == TIM13) {
        HAL_IncTick();
    }
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        controller.adc2_raw = hadc->Instance->JDR2; // Injected Rank2
        controller.adc1_raw = hadc->Instance->JDR1;

        if (state == MOTOR_MODE || state == VELOCITY_MODE || state == POSITION_MODE) {
            commutate(&controller, controller.theta_elec); // Run current loop
            controller.timeout++;
        }
    }
}



