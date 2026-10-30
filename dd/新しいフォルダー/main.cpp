#include "button.h"
#include "accounting.h"
#include <VMXPi.h>
#include <rclcpp/rclcpp.hpp>

#include <pthread.h>
#include <signal.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

int main(int argc, char* argv[])
{
    // ROS・VMXがスレッドを作る前に設定する
    sigset_t signals;
    sigemptyset(&signals);
    sigaddset(&signals, SIGINT);
    sigaddset(&signals, SIGTERM);

    const int result =
        pthread_sigmask(SIG_BLOCK, &signals, nullptr);

    if (result != 0) {
        std::cerr << "シグナル設定失敗: "
                  << std::strerror(result) << std::endl;
        return 1;
    }

    try {
        // シグナルはこのmainで受け取る
        rclcpp::init(
            argc,
            argv,
            rclcpp::InitOptions(),
            rclcpp::SignalHandlerOptions::None
        );

        auto vmx = std::make_shared<VMXPi>(true, 50);

        if (!vmx->IsOpen()) {
            throw std::runtime_error("VMX is not open.");
        }

        auto button_node = std::make_shared<BFE>(vmx);
        auto accounta = std::make_shared<SVE>(vmx, button_node);

        accounta->executorAdder(accounta);
        accounta->executorAdder(button_node);

        while (rclcpp::ok()) {
            const timespec timeout{0, 0};
            const int received =
                sigtimedwait(&signals, nullptr, &timeout);

            if (received == SIGINT || received == SIGTERM) {
                std::cerr << "main: 終了シグナル受信 "
                          << received << std::endl;
                break;
            }

            if (received == -1 &&
                errno != EAGAIN &&
                errno != EINTR) {
                throw std::runtime_error(
                    std::string("sigtimedwait: ") +
                    std::strerror(errno)
                );
            }

            accounta->executorSpin_some();
            accounta->finalizeRobotStop();

            std::this_thread::sleep_for(
                std::chrono::milliseconds(1)
            );
        }

        std::cerr << "main: SVE破棄開始" << std::endl;

        // VMXが開いている間に、LEDなどの終了処理を行う
        accounta.reset();
        button_node.reset();

        std::cerr << "main: VMX解放開始" << std::endl;
        vmx.reset();

        rclcpp::shutdown();

        std::cerr << "main: 終了処理完了" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "エラーが発生しました。"
                  << e.what() << std::endl;

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }

        return 1;
    }
}