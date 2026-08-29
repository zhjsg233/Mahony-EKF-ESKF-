/*
 * Justfloat.c
 *
 *  Created on: 
 *      Author: 34178
 */

#include "Justfloat.h"
#include "zf_device_wireless_uart.h"

const unsigned char tail[4]={0x00,0x00,0x80,0x7f};

void Justfloat_Init(void)
{
    wireless_uart_init();
}

void Justfloat_send_float(float f)
{
    unsigned char byte[4];
    FloatLongType fl;
    fl.f_data=f;

    byte[0]=(unsigned char)(fl.l_data);
    byte[1]=(unsigned char)(fl.l_data>>8);
    byte[2]=(unsigned char)(fl.l_data>>16);
    byte[3]=(unsigned char)(fl.l_data>>24);

    wireless_uart_send_buffer(byte,4);
}



void Justfloat_Send(void)
{

    wireless_uart_send_buffer(tail, 4);
}