//
// Created by SXF-Admin on 26-2-1.
//

#ifndef FOC_H
#define FOC_H

typedef struct SMC {
    float i_u, i_v, i_w; // Phase currents

    // 【新增】αβ轴的估算电流 (观测器状态)
    float i_alpha_est, i_beta_est;

    // 【新增】αβ轴的滑模输出 (高频项)
    float z_alpha, z_beta;

    // 【新增】αβ轴的反电动势 (平滑信号，核心输出)
    float e_alpha, e_beta;
    float es_u, es_v, es_w;
    float es_u_z, es_v_z, es_w_z;
    float theta;
    float v_d, v_q; // 观测得到的d-q 轴电压
    float v_alpha, v_beta;

    float es_u_z_last, es_v_z_last, es_w_z_last;
} smcParam_t;

typedef struct foc {
    float theta_elec; // 电角度 [rad]
    float v_des; // 期望电角速度 [rad/s]（注意：不是机械角速度）
    float v_d, v_q; // d-q 轴电压
    float v_u, v_v, v_w; // 输出的三相电压
    float real_voltage_u, real_voltage_v, real_voltage_w;
    float dtc_u, dtc_v, dtc_w; // 三相占空比（0～1）
    float v_bus; // 母线电压 [V]
    float i_u, i_v, i_w; // Phase currents
    float I_u_offset, I_v_offset, I_w_offset;
    float i_alpha, i_beta;
    int I_Calibration_flag;
    int zero_Calibration_flag;
    float i_d, i_q, i_q_filt, i_d_filt; // D/Q currents
} foc_t;

class Controller {
public:
    void open_loop(float sample_time);

    void SmcObserver();

    foc_t foc;
    smcParam_t smc_param;
};
#endif //FOC_H
