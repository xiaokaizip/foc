//
// Created by SXF-Admin on 26-2-6.
//

#ifndef MT6825_H
#define MT6825_H


typedef struct {
    float angle;
    bool mag_warning; //弱磁警报位，当感应到磁场太弱的时候，会触发警报
    bool over_speed; //超速报警位，当检测到旋转速度超速时，这一位置会置高电平
} mt6825_t;

extern mt6825_t mt6825;

void get_mt6825(mt6825_t *mt6825);

void print_mt6825(mt6825_t *mt6825);
#endif //MT6825_H
