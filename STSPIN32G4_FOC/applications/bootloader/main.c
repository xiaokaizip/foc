//
// Created by SXF-Admin on 26-1-14.
//

#include "main.h"

#include <string.h>

#include "usart.h"
#include "gpio.h"
#include "ymodem.h"
#include "menu.h"

extern void SystemClock_Config(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern pFunction JumpToApplication;
extern uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
static void IAP_Init(void);

void SystemClock_Config(void);

int main(void) {
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
    MX_USART1_UART_Init();

    /* Print formatted data */
    /* USER CODE BEGIN 2 */
    // 初始化 YMODEM 接收器


    /* USER CODE END 2 */


    /* Initialise Flash */
    FLASH_If_Init();
    // FLASH_If_Write(flashdestination, (uint32_t *) ramsource, packet_length / 8) == FLASHIF_OK

    /* Execute the IAP driver in order to reprogram the Flash */
    /* Display main menu */
    Main_Menu();

    /* Keep the user application running */

    /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
    if (((*(__IO uint32_t *) APPLICATION_ADDRESS) & 0x2FFE0000) == 0x20000000) {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t *) (APPLICATION_ADDRESS + 4);
        JumpToApplication = (pFunction) JumpAddress;
        __disable_irq();
        HAL_RCC_DeInit();
        HAL_DeInit();
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t *) APPLICATION_ADDRESS);
        JumpToApplication();
    }
    /* USER CODE END 3 */
}



