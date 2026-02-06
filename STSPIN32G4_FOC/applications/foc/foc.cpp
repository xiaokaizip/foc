//
// Created by SXF-Admin on 26-2-1.
//

#include "foc.h"
#include "math_ops.h"
#include "config.h"
#include "tim.h"
#include "SMC.h"

void abc(float theta, float d, float q, float *a, float *b, float *c) {
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *a = cf * d - sf * q; // Faster Inverse DQ0 transform
    *b = (0.86602540378f * sf - .5f * cf) * d - (-0.86602540378f * cf - .5f * sf) * q;
    *c = (-0.86602540378f * sf - .5f * cf) * d - (0.86602540378f * cf - .5f * sf) * q;
}


void dq0(float theta, float a, float b, float c, float *d, float *q) {
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c);
    ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w) {
    /// Space Vector Modulation ///
    /// u,v,w amplitude = v_bus for full modulation depth ///

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w)) * 0.5f;
    v_offset = 0;
    *dtc_u = fminf(fmaxf(((u - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);

    /*
    sinusoidal pwm
    *dtc_u = fminf(fmaxf((u/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf((v/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf((w/v_bus + .5f), DTC_MIN), DTC_MAX);
    */
}

void linearize_dtc(float *dtc) {
    /// linearizes the output of the inverter, which is not linear for small duty cycles ///
    float sgn = 1.0f - (2.0f * (*dtc < 0.0f));
    if (abs(*dtc) >= .01f) {
        *dtc = *dtc * .986f + .014f * sgn;
    } else {
        *dtc = 2.5f * (*dtc);
    }
}

void Controller::open_loop(float sample_time) {
    foc.theta_elec += sample_time * 1 * foc.v_des;
    foc.theta_elec = fmodf(foc.theta_elec, 2 * PI);


    abc(foc.theta_elec, foc.v_d, foc.v_q,
        &foc.v_u, &foc.v_v, &foc.v_w); //inverse dq0 transform on voltages
    svm(foc.v_bus, foc.v_u, foc.v_v, foc.v_w, &foc.dtc_u,
        &foc.dtc_v,
        &foc.dtc_w); //space vector modulation
    dq0(foc.theta_elec, foc.i_u, foc.i_v, foc.i_w, &foc.i_d,
        &foc.i_q);
    if (PHASE_ORDER) {
        // Check which phase order to use,
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - foc.dtc_u); // Write duty cycles
        TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - foc.dtc_v);
        TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - foc.dtc_w);
    } else {
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - foc.dtc_u);
        TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - foc.dtc_v);
        TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - foc.dtc_w);
    }
}

#define MAX_SMC_ERROR 5.5f
#define FILITER_C 200.0f
#define PWM_REEQ 16000.0f
float K_filter = 2.0f * 3.1415926f * FILITER_C / PWM_REEQ;
#define Kslide 20.0f
float alpha = 0.5f;

/**
 * @brief 辅助函数：Clarke 变换 (abc -> alpha/beta)
 */
static inline void clarke_transform(float a, float b, float c, float *alpha, float *beta) {
    *alpha = a;
    *beta = (b - c) * 0.57735026919f; // 1/sqrt(3)
}

/**
 * @brief 辅助函数：逆 Clarke 变换 (alpha/beta -> abc)
 *        注意：由于零序分量被忽略，c = -(a + b)
 */
static inline void inv_clarke_transform(float alpha, float beta, float *a, float *b, float *c) {
    *a = alpha;
    *b = -0.5f * alpha + 0.86602540378f * beta; // sqrt(3)/2
    *c = -0.5f * alpha - 0.86602540378f * beta;
}

/**
 * @brief 滑模观测器主函数
 *        内部在 αβ 坐标系下计算，但输入/输出仍使用三相结构。
 */
void Controller::SmcObserver() {
    // --- 1. 坐标变换：将三相电流和电压转换到 αβ 轴 ---
    float i_alpha, i_beta;
    clarke_transform(foc.i_u, foc.i_v, foc.i_w, &foc.i_alpha, &foc.i_beta);

    float v_alpha, v_beta;
    clarke_transform(foc.real_voltage_u, foc.real_voltage_v, foc.real_voltage_w, &v_alpha, &v_beta);

    // --- 2. Alpha 轴观测器 ---
    smc_param.i_alpha_est = F * smc_param.i_alpha_est + G * (foc.v_bus - smc_param.e_alpha - smc_param.z_alpha);
    float error_alpha = smc_param.i_alpha_est - foc.i_alpha;

    if (fabsf(error_alpha) < MAX_SMC_ERROR) {
        smc_param.z_alpha = error_alpha * Kslide
                            /
                            MAX_SMC_ERROR;
    } else {
        smc_param.z_alpha = (error_alpha > 0.0f)
                                ? Kslide
                                : -
                                Kslide;
    }
    smc_param.e_alpha += K_filter * (smc_param.z_alpha - smc_param.e_alpha);

    // --- 3. Beta 轴观测器 ---
    smc_param.i_beta_est = F * smc_param.i_beta_est + G * (foc.v_bus - smc_param.e_beta - smc_param.z_beta);
    float error_beta = smc_param.i_beta_est - foc.i_beta;

    if (fabsf(error_beta) < MAX_SMC_ERROR) {
        smc_param.z_beta = error_beta * Kslide
                           /
                           MAX_SMC_ERROR;
    } else {
        smc_param.z_beta = (error_beta > 0.0f) ? Kslide : -Kslide;
    }
    smc_param.e_beta += K_filter * (smc_param.z_beta - smc_param.e_beta);

    // --- 4. （可选）将估算的反电动势转换回三相形式 ---
    // 如果你的其他模块需要 es_u, es_v, es_w，可以在这里转换
    inv_clarke_transform(smc_param.e_alpha, smc_param.e_beta,
                         &smc_param.es_u, &smc_param.es_v, &smc_param.es_w);

    // --- 5. 计算转子电角度 ---
    smc_param.theta = atan2f(smc_param.e_beta, smc_param.e_alpha);

    // --- 6. （可选）将估算电流转换回三相形式，用于调试 ---
    inv_clarke_transform(smc_param.i_alpha_est, smc_param.i_beta_est,
                         &smc_param.i_u, &smc_param.i_v, &smc_param.i_w);
}
