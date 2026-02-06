//
// Created by SXF-Admin on 26-2-6.
//

#include "mt6825.h"
#include "main.h"
#include "spi.h"


void get_mt6825(mt6825_t *mt6825) {
    unsigned char tx[4] = {0x83}, rx[4];
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4,HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);

    uint16_t pc1;
    uint8_t pc2;

    //接收到的数据从rx[1]开始
    pc1 = rx[1] << 8 | rx[2];
    pc1 ^= pc1 >> 8;
    pc1 ^= pc1 >> 4;
    pc1 ^= pc1 >> 2;
    pc1 ^= pc1 >> 1;
    if (pc1 & 1) {
        return;
    }

    pc2 = rx[3] & 0xFC;
    pc2 ^= pc2 << 4;
    pc2 ^= pc2 << 2;
    pc2 ^= pc2 << 1;
    if (pc2 & 1) {
        return;
    }
    uint32_t raw = (static_cast<uint32_t>(rx[1]) << 10) | (static_cast<uint32_t>(rx[2]) << 2) | rx[3] >> 4;
    uint32_t angle_raw = (raw) & 0x3FFFF;
    mt6825->angle = static_cast<float>(angle_raw) * 360.0f / 262144.0f;
    mt6825->mag_warning = rx[1] & 0x02;
    mt6825->over_speed = rx[2] & 0x08;
}


