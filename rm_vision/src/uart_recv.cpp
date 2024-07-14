#include <iostream>
#include "../include/uart.hpp"
#include "../include/stop_watch.hpp"

int main()
{
    StopWatch stopWatch;

    // 下位机初始化通信
    Driver driver("/dev/ttyACM0", BaudRate::BAUD_115200);
	if (driver.open() != 0)
	{
		std::cout << "Uart open failed!" << std::endl;
		return false;
	}


    while(1)
    {
        // stopWatch.tic();
        // while(!driver.receiveStartSignal());
        // Gimbal_Position_t gimbal_pos;
        // driver.Unpack_Gimbal_Data(&gimbal_pos);
        // std::cout << gimbal_pos.cur_yaw*57.3f << "  " << gimbal_pos.cur_pitch*57.3f << "  " << (int)gimbal_pos.enemy_color<< std::endl;
        // std::cout << stopWatch.toc() << "ms" << std::endl;

        stopWatch.tic();
        while(!driver.receiveStartSignal());
        float quat[4], accel[3], gyro[3];
        driver.Unpack_Data(quat, accel, gyro);
        std::cout << accel[2] << std::endl;
    }
}
