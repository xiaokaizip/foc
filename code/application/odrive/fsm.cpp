//
// Created by SXF-Admin on 25-10-16.
//

#include "fsm.h"
#include "calibration.h"
#include "PositionSensor/PositionSensor.h"
#include "lwprintf/lwprintf.h"
#include "delay.h"
volatile int count = 0;
volatile int state = REST_MODE;
volatile int state_change;

extern PositionSensorAM5047 ps;
extern ControllerStruct controller;
extern PreferenceWriter prefs;

void calibrate(void) {
    order_phases(&ps, &controller, &prefs); // Check phase ordering
    calibrate(&ps, &controller, &prefs);
    state_change = 0;
}

void enter_menu_state(void) {
    lwprintf_printf("\n\r\n\r\n\r");
    lwprintf_printf(" Commands:\n\r");
    delayus(10);
    lwprintf_printf(" m - Motor Mode\n\r");
    delayus(10);
    lwprintf_printf(" c - Calibrate Encoder\n\r");
    delayus(10);
    lwprintf_printf(" s - Setup\n\r");
    delayus(10);
    lwprintf_printf(" e - Display Encoder\n\r");
    delayus(10);
    lwprintf_printf(" z - Set Zero Position\n\r");
    delayus(10);
    lwprintf_printf(" esc - Exit to Menu\n\r");
    delayus(10);
    state_change = 0;
}

void enter_torque_mode(void) {
    //gpio.enable.write(1);
    controller.ovp_flag = 0;
    reset_foc(&controller); // Tesets integrators, and other control loop parameters
    delayus(.001);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0; // Current Setpoints
    state_change = 0;

    lwprintf_printf("\n\r Entering Motor Mode \n\r");
}

void enter_setup_state(void) {
    lwprintf_printf("\n\r\n\r Configuration Options \n\r\n\n");
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
    delayus(10);
    lwprintf_printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    delayus(10);
    lwprintf_printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
    delayus(10);
    state_change = 0;
}

void print_encoder(void) {
    lwprintf_printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", ps.GetMechPosition(),
                    ps.GetElecPosition(), ps.GetRawPosition());
    //printf("%d\n\r", spi.GetRawPosition());
    delayus(100);
}

void fsm() {
    switch (state) {
        case REST_MODE: // Do nothing
            if (state_change) {
                enter_menu_state();
                TIM1->CCR3 = (PWM_ARR) * (1.0f - 0.5); // Write duty cycles
                TIM1->CCR2 = (PWM_ARR) * (1.0f - 0.5);
                TIM1->CCR1 = (PWM_ARR) * (1.0f - 0.5);
            }
            break;

        case CALIBRATION_MODE: // Run encoder calibration procedure
            if (state_change) {
                calibrate();
            }
            break;

        case MOTOR_MODE: // Run torque control
            if (state_change) {
                enter_torque_mode();
                count = 0;
            } else {
                if ((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0)) {
                    controller.i_d_ref = 0;
                    controller.i_q_ref = 0;
                    controller.kp = 0;
                    controller.kd = 0;
                    controller.t_ff = 0;
                }

                // torque_control(&controller);
                // commutate(&controller, controller.theta_elec); // Run current loop
                count++;
            }
            break;
        case SETUP_MODE:
            if (state_change) {
                enter_setup_state();
            }
            break;
        case ENCODER_MODE:
            print_encoder();
            break;
    }
}


