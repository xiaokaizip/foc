//
// Created by SXF-Admin on 26-2-6.
//

#ifndef ENCODER_H
#define ENCODER_H


#pragma once
#include <cstdint>
#include <cmath>
#include "spi.h"

class EncoderMT6825 {
public:
    static constexpr uint32_t CPR = 262144; // 18-bit

    bool init();
    bool update(float );   // Ts = 控制周期 (s)

    /* -------- ODrive 对齐输出 -------- */
    int32_t  count_in_cpr() const { return count_in_cpr_; }
    int64_t  shadow_count() const { return shadow_count_; }

    float pos_estimate() const { return pos_estimate_; }   // turns
    float vel_estimate() const { return vel_estimate_; }   // turns/s

    float phase() const { return phase_; }                 // rad
    float phase_vel() const { return phase_vel_; }         // rad/s

    /* -------- 参数 -------- */
    void set_pole_pairs(uint8_t pp) { pole_pairs_ = pp; }

private:
    /* -------- SPI -------- */
    bool read_raw(uint32_t &raw);

    /* -------- MT6825 -------- */
    uint32_t raw_pos_ = 0;
    uint32_t raw_pos_last_ = 0;

    /* -------- ODrive-style counts -------- */
    int32_t  count_in_cpr_ = 0;
    int64_t  shadow_count_ = 0;

    /* -------- PLL -------- */
    float pll_kp_ = 200.0f;
    float pll_ki_ = 4000.0f;

    float pos_estimate_counts_ = 0.0f;
    float vel_estimate_counts_ = 0.0f;

    float pos_estimate_ = 0.0f; // turns
    float vel_estimate_ = 0.0f; // turns/s

    /* -------- 插值 -------- */
    float interpolation_ = 0.5f;

    /* -------- FOC -------- */
    uint8_t pole_pairs_ = 7;
    float phase_ = 0.0f;
    float phase_vel_ = 0.0f;
};


#endif //ENCODER_H
