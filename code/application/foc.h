//
// Created by kade on 2025/9/24.
//

#ifndef FOC_FOC_H
#define FOC_FOC_H

#include <iostream>

class foc {
public:
    bool enable = false;
    float position = 0;
    float velocity = 0;
    float torque = 0;
};


#endif //FOC_FOC_H
