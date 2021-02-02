/*
 * bldc.h
 *
 *  Created on: Jan 25, 2021
 *      Author: jack
 *      Brief : Bldc controller using MAXON ESCON module (basically ESCON 24/2)
 */

#ifndef INC_BLDC_H_
#define INC_BLDC_H_

#define MAX_SPEED_RANGE 8000
#define MAX_CURRENT_RANGE 2.0

typedef struct
{
    float target_current;
    float target_speed;
    float actual_current;
    float actual_speed;
    _Bool enable;
    _Bool direction;
    uint16_t pwm_pulse;
} Bldc_State;

float bldc_get_speed(uint16_t adc_data);	//[rpm]
float bldc_get_current(uint16_t adc_data);	//[A]
uint16_t bldc_current_to_pulse(float target_current);	//[A]
uint16_t bldc_speed_to_pulse(float target_speed);	//[rpm]


#endif /* INC_BLDC_H_ */
