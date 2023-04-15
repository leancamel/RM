#include "rc_handoff.h"

#define CH4_NO_ACTION 0
#define CH4_SWITCH_UP 1
#define CH4_SWITCH_DOWN 2

#define CH4_HIGH_VALUE 600
#define CH4_LOW_VALUE 500

#define int_abs(x) ((x) > 0 ? (x) : (-x))

//将遥控器第四个通道的数据处理成开关量
//使用的时候注意 函数只能在一个周期里调用一次 第二次调用不起作用
//定义一个临时变量接收数据，不要else if中多次判断
static uint8_t rc_ch4_data_process(int16_t ch)
{
    static uint8_t flag = 0;

    if(int_abs(ch) > CH4_HIGH_VALUE && flag == 0)
    {
        flag = 1;
        return (ch > 0 ? CH4_SWITCH_DOWN : CH4_SWITCH_UP);
    }
    else if (int_abs(ch) < CH4_LOW_VALUE && flag == 1)
    {
        flag = 0;
    }

    return CH4_NO_ACTION;
}

//将rc_ch4_data_process的开关量转变为状态量
bool_t switch_is_fric_on(int16_t ch)
{
    static bool_t fric_flag = false;

    uint8_t temp = rc_ch4_data_process(ch);
    if(temp == CH4_SWITCH_UP && fric_flag == false)
    {
        fric_flag = true;
    }
    else if (temp == CH4_SWITCH_UP && fric_flag == true)
    {
        fric_flag = false;
    }

    return fric_flag;
}


bool_t switch_is_shoot(int16_t ch)
{
    static bool_t shoot_flag = 0;

    if(ch > CH4_HIGH_VALUE)
    {
        shoot_flag = 1;
    }
    else if (ch < CH4_LOW_VALUE)
    {
        shoot_flag = 0;
    }

    return shoot_flag;
}

