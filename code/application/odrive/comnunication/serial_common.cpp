// serial_comm.cpp
#include "serial_common.h"
#include <vector>
#include <cctype>
#include "structs.h"
#include "vofa_plus.h"
#include "lwprintf/lwprintf.h"
// UART handler（根据你的实际命名修改，如 huart4）
extern UART_HandleTypeDef huart4;

// 接收缓冲区和临时缓冲区
static uint8_t buffer[128];
static std::string rx_buffer;
static bool vofa_mode = false;
static uint32_t last_vofa_time = 0;
static const uint32_t VOFA_INTERVAL_US = 1000; // 1kHz = 1000us

// 外部变量（确保与你的主程序一致）


extern ControllerStruct controller;


// 状态字符串映射
State string_to_state(const std::string &s) {
    if (s == "REST") return REST_MODE;
    if (s == "CALIBRATION") return CALIBRATION_MODE;
    if (s == "MOTOR") return MOTOR_MODE;
    if (s == "SETUP") return SETUP_MODE;
    if (s == "ENCODER") return ENCODER_MODE;
    if (s == "OPEN_LOOP") return OPEN_LOOP_MODE;
    if (s == "VELOCITY_MODE") return VELOCITY_MODE;
    if (s == "POSITION_MODE") return POSITION_MODE;
    return REST_MODE; // 默认
}

// 解析单个参数赋值，如 kp:1.23f 或 kd:0.5
bool parse_param(const std::string &token) {
    auto pos = token.find(':');
    if (pos == std::string::npos) return false;

    std::string key = token.substr(0, pos);
    std::string val_str = token.substr(pos + 1);

    // 去除 f 后缀（如 1.0f）
    if (!val_str.empty() && val_str.back() == 'f') {
        val_str.pop_back();
    }

    char *end;
    float val = strtof(val_str.c_str(), &end);
    if (end == val_str.c_str()) return false; // 转换失败

    if (key == "kp") {
        controller.kp = val;
    } else if (key == "kd") {
        controller.kd = val;
    } else if (key == "p_des") {
        controller.p_des = val;
    } else if (key == "v_des") {
        controller.v_des = val;
    } else if (key == "t_ff") {
        controller.t_ff = val;
    } else if (key == "vel_kp") {
        controller.vel_kp = val;
    } else if (key == "vel_ki") {
        controller.vel_ki = val;
    } else if (key == "pos_kp") {
        controller.pos_kp = val;
    } else {
        return false;
    }
    return true;
}

// 解析参数行，支持 "kp:1.0 kd:2.0 ..." 或单个 "kp:1.0f"
void parse_params(const std::string &line) {
    std::string token;
    std::vector<std::string> tokens;

    for (char c: line) {
        if (std::isspace(c)) {
            if (!token.empty()) {
                tokens.push_back(token);
                token.clear();
            }
        } else {
            token += c;
        }
    }
    if (!token.empty()) tokens.push_back(token);

    bool success = false;
    for (const auto &t: tokens) {
        if (parse_param(t)) {
            success = true;
        }
    }

    if (success) {
        char buf[128];
        lwprintf_printf("Params updated: kp=%.3f, kd=%.3f, p_des=%.3f, v_des=%.3f, t_ff=%.3f\r\n",
                        controller.kp, controller.kd, controller.p_des, controller.v_des, controller.t_ff);
    } else {
        lwprintf_printf("Invalid parameter format\r\n");
    }
}

// 处理完整命令行
void handle_command(const std::string &cmd) {
    if (cmd.empty()) return;

    // 状态切换
    if (cmd == "REST" || cmd == "CALIBRATION" || cmd == "MOTOR" ||
        cmd == "SETUP" || cmd == "ENCODER" || cmd == "OPEN_LOOP" ||
        cmd == "VELOCITY_MODE" || cmd == "POSITION_MODE") {
        State new_state = string_to_state(cmd);
        if (state != new_state) {
            state = new_state;
            state_change = true;
            char buf[64];
            lwprintf_printf("State changed to %s\r\n", cmd.c_str());
        }
    }
    // 参数设置
    else if (cmd.find(':') != std::string::npos) {
        parse_params(cmd);
    }
    // VoFa+ 模式
    else if (cmd == "JustFloat") {
        vofa_mode = true;
        last_vofa_time = HAL_GetTick() * 1000; // us
        lwprintf_printf("VoFa+ mode started (1kHz)\r\n");
    } else if (cmd == "end") {
        vofa_mode = false;
        lwprintf_printf("VoFa+ mode stopped\r\n");
    } else {
        lwprintf_printf("%s Unknown command. Try: REST, MOTOR, kp:1.0, JustFloat, end\r\n", cmd.c_str());
    }
}

// 串口初始化
void serial_init() {
    // 启动串口接收中断
    HAL_UART_Receive_DMA(&huart4, buffer, 128);
    // 开启空闲中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_TC);
    rx_buffer.clear();
}

// // 串口接收完成回调（在中断中调用）
// extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart == &huart4) {
//         if (rx_byte == '\r' || rx_byte == '\n') {
//             if (!rx_buffer.empty()) {
//                 // 将接收到的命令放入主循环处理
//                 handle_command(rx_buffer);
//                 rx_buffer.clear();
//             }
//         } else {
//             // 防止缓冲区过大
//             if (rx_buffer.size() < 128) {
//                 rx_buffer += (char) rx_byte;
//             }
//         }
//         // 重新开启下一次中断接收
//         HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
//     }
// }
extern "C" void UART4_IRQHandler(void) {
    /* USER CODE BEGIN UART4_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_IDLE)) {
        // 清除 idle 标志（必须先读SR再读DR）
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);

        // 停止 DMA 接收以便处理数据（如果你用了 DMA）
        HAL_UART_DMAStop(&huart4);

        // 计算接收到的字节数（假设 buffer 是 128 字节）
        uint32_t len = 128 - __HAL_DMA_GET_COUNTER(huart4.hdmarx);

        if (len > 0) {
            // 将 buffer 前 len 字节转为 string 并添加到 rx_buffer
            rx_buffer.append(reinterpret_cast<char *>(buffer), len);

            // 查找是否有完整行（\r\n 结尾）
            size_t pos;
            while ((pos = rx_buffer.find_first_of("\r\n")) != std::string::npos) {
                std::string cmd = rx_buffer.substr(0, pos);
                handle_command(cmd);
                rx_buffer.erase(0, pos + 1); // 删除已处理部分（含 \r 或 \n）
            }

            // 保留未完成的部分（比如只收到一半命令）
        }

        // 重新启动 DMA 接收
        HAL_UART_Receive_DMA(&huart4, buffer, 128);
    }
    /* USER CODE END UART4_IRQn 0 */
    HAL_UART_IRQHandler(&huart4);
    /* USER CODE BEGIN UART4_IRQn 1 */

    /* USER CODE END UART4_IRQn 1 */
}

// 主循环中调用此函数处理 VoFa+ 发送
void serial_process() {
    if (vofa_mode) {
        uint32_t now = HAL_GetTick() * 1000 + (HAL_GetTick() % 1000) * 1000; // 近似 us
        if (now - last_vofa_time >= VOFA_INTERVAL_US) {
            last_vofa_time = now;
            send_to_vofa(); // 假设 ps 是你要发送的数据结构
        }
    }
}

