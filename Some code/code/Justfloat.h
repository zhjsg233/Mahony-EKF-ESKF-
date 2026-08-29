/*
 * Justfloat.h
 *
 *  Created on: 2024?3?4?
 *      Author: 34178
 */

#ifndef CODE_JUSTFLOAT_H_
#define CODE_JUSTFLOAT_H_

typedef union
{
        float f_data;
        unsigned long l_data;
} FloatLongType;

void Justfloat_Init(void);
void Justfloat_send_float(float f);
void Justfloat_Send(void);

#endif /* CODE_JUSTFLOAT_H_ */