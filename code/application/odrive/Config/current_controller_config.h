#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

// Current controller///
#define K_D .05f                    // Loop gain,  Volts/Amp
#define K_Q .05f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.002f                // PI zero, in radians per sample
#define KI_Q 0.002f                // PI zero, in radians per sample
#define V_BUS 24.0f                 // Volts
#define OVERMODULATION 0.15f        // 1.0 = no overmodulation

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples

//Observer//
#define PS_DT 0.001f    //位置传感器的数据采用周期
#define DT 0.0001f
#define K_O 0.02f


#endif
