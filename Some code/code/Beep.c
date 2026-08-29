#include "zf_common_headfile.h"

void Beep_init(void)
{
    gpio_init(P19_4,GPO,0,GPO_PUSH_PULL);
}

void Beep_On(void)
{
    gpio_set_level(P19_4,1);
}

void Beep_Off(void)
{
    gpio_set_level(P19_4,0);
}