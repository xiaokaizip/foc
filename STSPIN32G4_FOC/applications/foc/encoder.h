//
// Created by SXF-Admin on 26-2-6.
//

#ifndef ENCODER_H
#define ENCODER_H

typedef struct {
    float elec_angle;
    float elecangle_last;
    float elecangle_offset;
    float mech_angle;
    float mech_ofset;
    float count;
    float pairs;
    float elec_speed;

    float speed_last;
} encoder_t;


#endif //ENCODER_H
