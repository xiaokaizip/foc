//
// Created by SXF-Admin on 25-10-22.
//

#include "can_simple.h"
#include "structs.h"
#include <cstring> // for memcpy
#include <array> // 确保包含
#include "fsm.h"
// 假设 ControllerStruct 在 structs.h 中定义
extern ControllerStruct controller;

/**
 * @brief: float_to_uint: 将浮点数转换为无符号整型
 * @param[in]: x_float: 待转换的浮点数
 * @param[in]: x_min: 范围最小值
 * @param[in]: x_max: 范围最大值
 * @param[in]: bits: 目标无符号整型的位数
 * @retval: 无符号整型值
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    if (x_float < x_min) x_float = x_min;
    if (x_float > x_max) x_float = x_max;
    float span = x_max - x_min;
    return (int) ((x_float - x_min) * ((float) ((1 << bits) - 1)) / span);
}

/**
 * @brief: uint_to_float: 无符号整型转换为浮点数
 * @param[in]: x_int: 待转换的无符号整型
 * @param[in]: x_min: 范围最小值
 * @param[in]: x_max: 范围最大值
 * @param[in]: bits: 无符号整型的位数
 * @retval: 浮点数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + x_min;
}

extern int state;
extern int state_change;
/**
 * @brief: do_command: 解析来自主机的CAN控制命令
 * @param[in]: id: 接收到的CAN帧ID
 * @param[in]: data: 接收到的8字节CAN数据
 * @details: 该函数根据DM-H3510电机的不同模式和命令，解析data中的控制参数。
 *           支持MIT模式、位置速度模式、速度模式、力位混控模式以及特殊命令（使能/失能等）。
 */
void CANSimple::do_command(unsigned short id, unsigned char *data) {
    // 步骤1: 判断是特殊命令还是控制命令
    // 特殊命令: 8字节全FF，最后1字节为特定值
    bool is_special_cmd = true;
    for (int i = 0; i < 7; ++i) {
        if (data[i] != 0xFF) {
            is_special_cmd = false;
            break;
        }
    }

    if (is_special_cmd) {
        // 处理特殊命令
        switch (data[7]) {
            case 0xFC:
                state = MOTOR_MODE;
                state_change = true;
                // 使能命令
                // 可以在这里设置一个使能标志
                // example: motor_enable_flag = true;
                break;
            case 0xFD:
                state = REST_MODE;
                state_change = true;

                // 失能命令
                // 可以在这里设置一个失能标志
                // example: motor_enable_flag = false;
                break;
            case 0xFE:
                // 保存位置零点命令
                // 可以在这里触发保存当前编码器位置为零点的操作
                // example: save_zero_position();
                break;
            case 0xFB:
                // 清除错误命令
                // 可以在这里清除电机的错误状态
                // example: clear_motor_error();
                break;
            default:
                // 未知特殊命令
                break;
        }
        return; // 特殊命令处理完毕
    }

    // 步骤2: 判断控制模式 (通过ID)
    // 根据说明书第9页，不同模式使用不同的ID偏移量
    uint16_t mode_offset = id & 0xF00; // 取ID的高4位
    uint16_t motor_id = id & 0x0FF; // 取ID的低8位，即实际电机ID

    // 检查motor_id是否与本电机的ESC_ID匹配
    // 这里假设本电机的ID是 CAN_RECEIVE_ID
    if (motor_id == CAN_RECEIVE_ID) {
        // state_change = true;

        // 步骤3: 根据mode_offset解析对应模式的数据
        switch (mode_offset) {
            case 0x000: // MIT模式
            {
                controller.timeout = 0;
                // D[0] 和 D[1]: 16位位置给定 (p_des)
                unsigned short temp_pos = (data[0] << 8) | data[1];
                // D[2] 和 D[3]: 12位速度给定 (v_des)，D[2]全部 + D[3]高4位
                unsigned short temp_vel = (data[2] << 4) | (data[3] >> 4);
                // D[3] 和 D[4]: 12位Kp，D[3]低4位 + D[4]全部
                unsigned short temp_kp = ((data[3] & 0x0F) << 8) | data[4];
                // D[5] 和 D[6]: 12位Kd，D[5]全部 + D[6]高4位
                unsigned short temp_kd = (data[5] << 4) | ((data[6] & 0xF0) >> 4);
                // D[6] 和 D[7]: 12位前馈扭矩(t_ff)，D[6]低4位 + D[7]全部
                unsigned short temp_tff = ((data[6] & 0xF) << 8) | data[7];

                // 转换为浮点数
                // 范围依据说明书及常见应用
                controller.p_des = uint_to_float(temp_pos, -12.5f, 12.5f, 16);
                controller.v_des = uint_to_float(temp_vel, -45.0f, 45.0f, 12);
                controller.kp = uint_to_float(temp_kp, 0, 500.0f, 12) / 10.0f; // Kp [0, 500]
                controller.kd = uint_to_float(temp_kd, 0, 5.0f, 12) / 1000.0f; // Kd [0, 5]
                controller.t_ff = uint_to_float(temp_tff, -0.2f, 0.2f, 12); // t_ff [-0.45, 0.45] Nm

                // 设置模式标志
                // controller.mode = CTRL_MODE_MIT;
                break;
            }

            case 0x100: // 位置速度模式
            {
                // 数据格式: p_des(4字节) + v_des(4字节)
                // 符合IEEE 754标准，小端模式
                std::memcpy(&controller.p_des, data, 4); // D[0]-D[3]
                std::memcpy(&controller.v_des, &data[4], 4); // D[4]-D[7]

                // t_ff, kp, kd 在此模式下通常不使用或忽略
                // controller.t_ff = 0.0f;
                // controller.kp = 0.0f;
                // controller.kd = 0.0f;

                // 设置模式标志
                // controller.mode = CTRL_MODE_POS_SPEED;
                break;
            }

            case 0x200: // 速度模式
            {
                // 数据格式: v_des(4字节) + 保留(4字节)
                std::memcpy(&controller.v_des, data, 4); // D[0]-D[3]
                // D[4]-D[7] 未使用

                // p_des, t_ff, kp, kd 在此模式下通常不使用或忽略
                // controller.p_des = 0.0f;
                // controller.t_ff = 0.0f;
                // controller.kp = 0.0f;
                // controller.kd = 0.0f;

                // 设置模式标志
                // controller.mode = CTRL_MODE_SPEED;
                break;
            }

            case 0x300: // 力位混控模式
            {
                // 数据格式: p_des(4字节) + v_des(2字节) + i_des(2字节)
                // p_des: float, 小端
                std::memcpy(&controller.p_des, data, 4); // D[0]-D[3]
                // v_des: 无符号16位整数，放大100倍，范围0-10000对应0-100rad/s
                uint16_t v_raw = (data[5] << 8) | data[4]; // 注意：小端，低位在前
                controller.v_des = (float) v_raw / 100.0f;
                // i_des: 无符号16位整数，放大10000倍，范围0-10000对应0-1.0
                uint16_t i_raw = (data[7] << 8) | data[6]; // 注意：小端，低位在前
                controller.t_ff = (float) i_raw / 10000.0f; // 重用t_ff字段存储i_des

                // kp, kd 在此模式下通常不使用或忽略
                // controller.kp = 0.0f;
                // controller.kd = 0.0f;

                // 设置模式标志
                // controller.mode = CTRL_MODE_FORCE_POS_MIX;
                break;
            }

            default:
                // 未知模式
                break;
        }
    }
}

std::array<unsigned char, 8> CANSimple::packet_data() {
    std::array<unsigned char, 8> data = {0}; // 使用 std::array


    unsigned short temp_pos = float_to_uint(controller.theta_mech, -12.5f, 12.5f, 16);
    unsigned short temp_v_des = float_to_uint(20.0f, -45.0f, 45.0f, 12);
    unsigned short temp_t_ff = float_to_uint(controller.i_q, -18.0f, 18.0f, 16);
    data[0] = CAN_SEND_ID | 8 << 4;
    data[1] = temp_pos >> 8;
    data[2] = temp_pos & 0xFF;
    data[3] = temp_v_des >> 4;
    data[4] = (temp_v_des & 0x0f << 4) | (temp_t_ff >> 8) & 0x0F;
    data[5] = temp_t_ff & 0xFF;
    data[6] = 30;
    data[7] = 30;
    return data;
}
