//
// Created by SXF-Admin on 26-1-14.
//

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "lwprintf/lwprintf.h"
extern  void SystemClock_Config(void);

static int
lwprintf_my_out_func(int ch, lwprintf_t* p) {
    uint8_t c = (uint8_t)ch;

    /* Don't print zero */
    if (c == '\0') {
        return ch;
    }
    HAL_UART_Transmit(&huart1, &c, 1, 100);
    return ch;
}

int main(void)
{

    /* USER CODE BEGIN 1 */

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
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    lwprintf_init(lwprintf_my_out_func);

    /* Print formatted data */
    lwprintf_printf("My first string: %s\r\n", "Hello world");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        lwprintf_printf("hello world\n");
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);

        HAL_Delay(1000);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}



