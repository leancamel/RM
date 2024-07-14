#include <iostream>
#include "uart.hpp"
#include "stop_watch.hpp"
#include <signal.h>
#include <thread>

void callbackSignal(int signum);

Driver driver("/dev/ttyACM0", BaudRate::BAUD_115200);
int main()
{
    StopWatch stopWatch;

    // 下位机初始化通信
	if (driver.open() != 0)
	{
		std::cout << "Uart open failed!" << std::endl;
		return false;
	}
    signal(SIGINT, callbackSignal); // 程序退出信号

    while(1)
    {
        float yaw = 0.0f;
        float pitch = 0.0f;
        std::cout << "DEG: yaw\tpitch" << std::endl;

        std::cin >> yaw >> pitch;
        yaw /= 57.3;
        pitch /= 57.3;

        stopWatch.tic();
        driver.shoot_angle(yaw, pitch, 20);        
        std::cout << stopWatch.toc() << "ms" << std::endl;
    }
}

/**
 * @brief 系统信号回调函数：系统退出
 * @param signum 信号量
 */
void callbackSignal(int signum)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));// 延时等待串口发送完毕
    driver.shoot_angle(0, 0, 0);
    std::cout << "gimbal stop moving--->" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));// 延时等待串口发送完毕
    std::cout << "====System Exit!!!  -->  ShootStopping! " << signum << std::endl;
    exit(signum);
}

