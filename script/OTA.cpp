#include <SerialPort.h>
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

using namespace LibSerial;

volatile bool keepRunning = true;

void signalHandler(int signum) {
    keepRunning = false;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "用法: " << argv[0] << " <串口设备>\n"
                  << "例如: " << argv[0] << " /dev/ttyUSB0\n";
        return 1;
    }

    std::string port = argv[1];
    SerialPort serialPort;

    // 注册 Ctrl+C 信号处理
    signal(SIGINT, signalHandler);

    try {
        serialPort.Open(port);
        serialPort.SetBaudRate(BaudRate::BAUD_115200);
        serialPort.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serialPort.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        serialPort.SetParity(Parity::PARITY_NONE);
        serialPort.SetStopBits(StopBits::STOP_BITS_1);

        std::cout << "✅ 已连接到 " << port << " @ 115200\n";
        std::cout << "正在监听数据...（按 Ctrl+C 退出）\n\n";

        // 清空输入缓冲区
        serialPort.FlushInputBuffer();

        char buffer[1024];
        while (keepRunning) {
            size_t available = serialPort.GetNumberOfBytesAvailable();
            if (available > 0) {
                size_t bytesRead = serialPort.Read(buffer, std::min(available, sizeof(buffer) - 1), 100); // 100ms 超时
                buffer[bytesRead] = '\0';
                std::cout << buffer;
                std::cout.flush();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        serialPort.Close();
        std::cout << "\n\n� 用户中断，退出监听。\n";

    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}


