// FlashWriter.cpp
#include "FlashWriter.h"
#include <cstring>  // 用于 memcpy

FlashWriter::FlashWriter(uint8_t sector) {
    if (sector > 7) sector = 7;
    __sector = sector;
    __base = __SECTOR_ADDRS[sector];
    __ready = false;
}

bool FlashWriter::ready() {
    return __ready;
}

void FlashWriter::open() {
    // 解锁 Flash
    if (HAL_FLASH_Unlock() != HAL_OK) {
        __ready = false;
        return;
    }

    // 配置擦除操作
    FLASH_EraseInitTypeDef eraseInitStruct = {0};
    uint32_t sectorError = 0;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.Sector = __sector;
    eraseInitStruct.NbSectors = 1;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 适用于 2.7V~3.6V

    // 执行擦除
    if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK) {
        __ready = false;
        HAL_FLASH_Lock(); // 擦除失败也要上锁
        return;
    }

    __ready = true;
}

void FlashWriter::write(uint32_t index, int x) {
    uint32_t address = __base + 4 * index;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint32_t) x) != HAL_OK) {
        // 可选：错误处理，例如设置 __ready = false
    }
}

void FlashWriter::write(uint32_t index, unsigned int x) {
    uint32_t address = __base + 4 * index;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, x) != HAL_OK) {
        // 可选错误处理
    }
}

void FlashWriter::write(uint32_t index, float x) {
    uint32_t data;
    std::memcpy(&data, &x, sizeof(x)); // 安全转换 float -> uint32_t
    uint32_t address = __base + 4 * index;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) != HAL_OK) {
        // 可选错误处理
    }
}

void FlashWriter::close() {
    HAL_FLASH_Lock();
    __ready = false;
}

// 读取函数：直接内存访问，无需解锁
int flashReadInt(uint32_t sector, uint32_t index) {
    return *reinterpret_cast<volatile int *>(
        __SECTOR_ADDRS[sector] + 4 * index
    );
}

uint32_t flashReadUint(uint32_t sector, uint32_t index) {
    return *reinterpret_cast<volatile uint32_t *>(
        __SECTOR_ADDRS[sector] + 4 * index
    );
}

float flashReadFloat(uint32_t sector, uint32_t index) {
    return *reinterpret_cast<volatile float *>(
        __SECTOR_ADDRS[sector] + 4 * index
    );
}
