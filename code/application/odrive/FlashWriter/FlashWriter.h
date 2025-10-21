// FlashWriter.h
#ifndef __FLASHWRITER_H
#define __FLASHWRITER_H

#include "main.h"  // 包含 stm32f4xx_hal.h 和 HAL 配置

/* Base address of the Flash sectors (from RM0090) */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* 16 KB */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* 16 KB */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* 16 KB */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* 16 KB */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* 64 KB */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* 128 KB */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* 128 KB */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* 128 KB */

// 映射扇区编号（HAL 使用 0~7）
static const uint32_t __SECTOR_ADDRS[] = {
    ADDR_FLASH_SECTOR_0,
    ADDR_FLASH_SECTOR_1,
    ADDR_FLASH_SECTOR_2,
    ADDR_FLASH_SECTOR_3,
    ADDR_FLASH_SECTOR_4,
    ADDR_FLASH_SECTOR_5,
    ADDR_FLASH_SECTOR_6,
    ADDR_FLASH_SECTOR_7
};

// 注意：原代码中 __SECTORS 有笔误（两个 Sector_6），我们不需要这个映射表
// 因为 HAL 直接用 sector number (0~7) 即可

class FlashWriter {
public:
    FlashWriter(uint8_t sector);

    void open();

    bool ready();

    void write(uint32_t index, int x);

    void write(uint32_t index, unsigned int x);

    void write(uint32_t index, float x);

    void close();

private:
    uint32_t __base;
    uint8_t __sector; // 扇区编号 0~7
    bool __ready;
};

// 独立读取函数
int flashReadInt(uint32_t sector, uint32_t index);

uint32_t flashReadUint(uint32_t sector, uint32_t index);

float flashReadFloat(uint32_t sector, uint32_t index);

#endif // __FLASHWRITER_H
