//
// Created by kade on 2025/9/20.
//

#include "drv8301.h"
#include "iostream"
#include <string>
#include "lwprintf/lwprintf.h"
#include "spi.h"


#include "delay.h"


#include "spi.h"

bool Drv8301::config(float requested_gain, float *actual_gain) {
    // Calculate gain setting: Snap down to have equal or larger range as
    // requested or largest possible range otherwise

    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    uint16_t gain_setting = 3;
    float gain_choices[] = {10.0f, 20.0f, 40.0f, 80.0f};
    while (gain_setting && (gain_choices[gain_setting] > requested_gain)) {
        gain_setting--;
    }

    if (actual_gain) {
        *actual_gain = gain_choices[gain_setting];
    }

    RegisterFile new_config;

    new_config.control_register_1 =
            (23 << 6) // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
            | (0b01 << 4) // OCP_MODE: latch shut down
            | (0b0 << 3) // 6x PWM mode
            | (0b0 << 2) // don't reset latched faults
            | (0b00 << 0); // gate-drive peak current: 1.7A

    new_config.control_register_2 =
            (0b0 << 6) // OC_TOFF: cycle by cycle
            | (0b00 << 4) // calibration off (normal operation)
            | (gain_setting << 2) // select gain
            | (0b00 << 0); // report both over temperature and over current on nOCTW pin

    bool regs_equal = (regs_.control_register_1 == new_config.control_register_1)
                      && (regs_.control_register_2 == new_config.control_register_2);

    if (!regs_equal) {
        regs_ = new_config;
        state_ = kStateUninitialized;
        HAL_GPIO_WritePin(enable_gpio_port_, enable_gpio_pin_, GPIO_PIN_RESET);
    }

    return true;
}

bool Drv8301::init() {
    uint16_t value_control_register_1;
    uint16_t value_control_register_2;


    DEBUG_PRINT("drv8301 init\n\r");
    if (state_ == kStateReady) {
        return true;
    }

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.
    HAL_GPIO_WritePin(enable_gpio_port_, enable_gpio_pin_, GPIO_PIN_RESET);
    delayus(40); // mimumum pull-down time for full reset: 20us
    state_ = kStateUninitialized; // make is_ready() ignore transient errors before registers are set up
    HAL_GPIO_WritePin(enable_gpio_port_, enable_gpio_pin_, GPIO_PIN_SET);

    HAL_Delay(10); // t_spi_ready, max = 10ms

    // Write current configuration
    bool wrote_regs = write_reg(kRegNameControl1, regs_.control_register_1)
                      && write_reg(kRegNameControl1, regs_.control_register_1)
                      && write_reg(kRegNameControl1, regs_.control_register_1)
                      && write_reg(kRegNameControl1, regs_.control_register_1)
                      && write_reg(kRegNameControl1,
                                   regs_.control_register_1)
                      // the write operation tends to be ignored if only done once (not sure why)
                      && write_reg(kRegNameControl2, regs_.control_register_2);
    if (!wrote_regs) {
        return false;
    }

    // Wait for configuration to be applied
    delayus(100);
    state_ = kStateStartupChecks;
    read_reg(kRegNameControl1, &value_control_register_1);
    read_reg(kRegNameControl2, &value_control_register_2);
    bool is_read_regs = (value_control_register_1 == regs_.control_register_1)
                        && (value_control_register_2 == regs_.control_register_2);
    DEBUG_PRINT("drv8301 read controller regs data:%d %d \n\r", value_control_register_1, value_control_register_2);
    if (!is_read_regs) {
        return false;
    }

    if (get_error() != FaultType_NoFault) {
        return false;
    }

    //    // There could have been an nFAULT edge meanwhile. In this case we shouldn't
    //    // consider the driver ready.
    //    CRITICAL_SECTION() {
    //
    //    }
    if (state_ == kStateStartupChecks) {
        state_ = kStateReady;
    }
    return state_ == kStateReady;
}

void Drv8301::do_checks() {
    if (state_ != kStateUninitialized && !HAL_GPIO_ReadPin(nfault_gpio_port_, nfault_gpio_pin_)) {
        state_ = kStateUninitialized;
    }
    //    if (state_ != kStateUninitialized && !  nfault_gpio_.read()) {
    //        state_ = kStateUninitialized;
    //    }
}

bool Drv8301::is_ready() {
    return state_ == kStateReady;
}


Drv8301::FaultType_e Drv8301::get_error() {
    uint16_t fault1, fault2;

    if (!read_reg(kRegNameStatus1, &fault1) ||
        !read_reg(kRegNameStatus2, &fault2)) {
        return (FaultType_e) 0xffffffff;
    }

    return (FaultType_e) ((uint32_t) fault1 | ((uint32_t) (fault2 & 0x0080) << 16));
}

bool Drv8301::read_reg(const RegName_e regName, uint16_t *data) {
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);
    HAL_StatusTypeDef status;
    HAL_GPIO_WritePin(ncs_gpio_port_, ncs_gpio_pin_, GPIO_PIN_RESET);
    delayus(10);

    status = HAL_SPI_Transmit(spi_arbiter_, (uint8_t *) &tx_buf_, 1, HAL_MAX_DELAY);
    delayus(10);


    HAL_GPIO_WritePin(ncs_gpio_port_, ncs_gpio_pin_, GPIO_PIN_SET);

    if (status != HAL_OK) {
        return false;
    }

    delayus(1000);

    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);
    rx_buf_ = 0xffff;
    HAL_GPIO_WritePin(ncs_gpio_port_, ncs_gpio_pin_, GPIO_PIN_RESET);
    delayus(10);

    status = HAL_SPI_TransmitReceive(spi_arbiter_, (uint8_t *) &tx_buf_, (uint8_t *) &rx_buf_, 1, HAL_MAX_DELAY);

    delayus(10);

    HAL_GPIO_WritePin(ncs_gpio_port_, ncs_gpio_pin_, GPIO_PIN_SET);

    delayus(1);
    if (status == HAL_OK) {
        DEBUG_PRINT("drv8301 read 0x%x reg successfully, read reg:0x%x\n\r", regName, rx_buf_);
    } else {
        DEBUG_PRINT("drv8301 read 0x%x reg fuid, read reg:0x%x\n\r", regName, rx_buf_);
    }
    if (rx_buf_ == 0xbeef) {
        return false;
    }

    if (data) {
        *data = rx_buf_ & 0x07FF;
    }

    return true;
}

bool Drv8301::write_reg(const RegName_e regName, const uint16_t data) {
    // Do blocking write
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Write, regName, data);
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(ncs_gpio_port_, ncs_gpio_pin_, GPIO_PIN_RESET);
    delayus(10);

    status = HAL_SPI_TransmitReceive(spi_arbiter_, (uint8_t *) &tx_buf_, (uint8_t *) &rx_buf_, 1, HAL_MAX_DELAY);
    delayus(10);

    HAL_GPIO_WritePin(ncs_gpio_port_, ncs_gpio_pin_, GPIO_PIN_SET);

    if (status == HAL_OK) {
        DEBUG_PRINT("drv8301 write 0x%x reg value 0x%x successfully\n\r", regName, data);
        DEBUG_PRINT("drv8301 read reg value 0x%x successfully\n\r", rx_buf_);
    }

    return true;
}





