//
// Created by SXF-Admin on 25-10-22.
//

#ifndef CAN_SIMPLE_H
#define CAN_SIMPLE_H
#include <cstdint>


class CANSimple {
public:
    CANSimple();

    void do_command(Axis &axis, const can_Message_t &cmd);

private:
}
#endif //CAN_SIMPLE_H
