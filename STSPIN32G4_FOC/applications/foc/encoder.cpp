//
// Created by SXF-Admin on 26-2-6.
//

#include "encoder.h"

static uint8_t parity(uint32_t x) {
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1;
}

bool EncoderMT6825::read_raw(uint32_t &raw) {
    uint8_t tx[4] = {0x83, 0, 0, 0};
    uint8_t rx[4] = {0};

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    uint16_t pc1 = (rx[1] << 8) | rx[2];
    if (parity(pc1)) return false;

    uint8_t pc2 = rx[3] & 0xFC;
    if (parity(pc2)) return false;

    raw =
            ((uint32_t)rx[1] << 10) |
            ((uint32_t)rx[2] << 2)  |
            (rx[3] >> 4);

    raw &= 0x3FFFF;
    return true;
}

bool EncoderMT6825::update(float Ts) {
    uint32_t raw;
    if (!read_raw(raw)) {
        return false;
    }

    raw_pos_ = raw;

    /* -------- delta_enc（ODrive 同款）-------- */
    int32_t delta_enc = (int32_t)raw_pos_ - (int32_t)raw_pos_last_;

    if (delta_enc >  (int32_t)(CPR / 2)) delta_enc -= CPR;
    if (delta_enc < -(int32_t)(CPR / 2)) delta_enc += CPR;

    raw_pos_last_ = raw_pos_;

    /* -------- 累加 count -------- */
    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;

    if (count_in_cpr_ >= (int32_t)CPR) count_in_cpr_ -= CPR;
    if (count_in_cpr_ < 0)             count_in_cpr_ += CPR;

    /* -------- PLL 预测 -------- */
    pos_estimate_counts_ += Ts * vel_estimate_counts_;

    /* -------- 相位探测 -------- */
    float delta_pos = (float)(count_in_cpr_) - pos_estimate_counts_;

    if (delta_pos >  (float)(CPR / 2)) delta_pos -= CPR;
    if (delta_pos < -(float)(CPR / 2)) delta_pos += CPR;

    /* -------- PLL 校正 -------- */
    pos_estimate_counts_ += Ts * pll_kp_ * delta_pos;
    vel_estimate_counts_ += Ts * pll_ki_ * delta_pos;

    /* -------- 插值 -------- */
    if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        interpolation_ += Ts * vel_estimate_counts_;
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }

    float interp_count = (float)count_in_cpr_ + interpolation_;

    /* -------- 输出（turns）-------- */
    pos_estimate_ = pos_estimate_counts_ / (float)CPR;
    vel_estimate_ = vel_estimate_counts_ / (float)CPR;

    /* -------- FOC 电角度 -------- */
    float elec_rad_per_cnt =
            pole_pairs_ * 2.0f * M_PI / (float)CPR;

    phase_ =
            fmodf(interp_count * elec_rad_per_cnt, 2.0f * M_PI);

    phase_vel_ =
            vel_estimate_ * 2.0f * M_PI * pole_pairs_;

    return true;
}
