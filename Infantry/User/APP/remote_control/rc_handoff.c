#include "rc_handoff.h"

#define CH4_HIGH_VALUE 600
#define CH4_LOW_VALUE 500

#define int_abs(x) ((x) > 0 ? (x) : (-x))

//将遥控器第四个通道的数据处理成开关量
//使用的时候注意 函数只能在一个周期里调用一次 第二次调用不起作用
//定义一个临时变量接收数据，不要else if中多次判断
uint8_t rc_ch4_data_process(int16_t ch)
{
    static uint8_t flag = 0;

    if(int_abs(ch) > CH4_HIGH_VALUE && flag == 0)
    {
        flag = 1;
        // ch4_process = ch > 0 ? AUTO_ON : AUTO_OFF;
        return (ch > 0 ? CH4_SWITCH_DOWN : CH4_SWITCH_UP);
    }
    else if (int_abs(ch) < CH4_LOW_VALUE && flag == 1)
    {
        flag = 0;
    }

    return CH4_NO_ACTION;
}

