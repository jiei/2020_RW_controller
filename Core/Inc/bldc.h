/*
 * bldc.h
 *
 *  Created on: Jan 25, 2021
 *      Author: jack
 *      Brief : Bldc controller using MAXON ESCON module (basically ESCON 24/2)
 */

#ifndef INC_BLDC_H_
#define INC_BLDC_H_

#define MAX_SPEED_RANGE 8000	//速度レンジ -8000~8000[rpm]
#define MAX_CURRENT_RANGE 2.0	//電流レンジ -2.0~2.0[A]

typedef struct
{
    float target_current;	//目標電流値
    float target_speed;		//目標速度
    float actual_current;	//測定電流値
    float actual_speed;		//測定速度
    _Bool enable;			//enable or not
    _Bool direction;
    uint16_t pwm_pulse;
} Bldc_State;

float bldc_get_speed(uint16_t adc_data);	//[rpm]
float bldc_get_current(uint16_t adc_data);	//[A]
uint16_t bldc_current_to_pulse(float target_current);	//[A]
uint16_t bldc_speed_to_pulse(float target_speed);	//[rpm]


#endif /* INC_BLDC_H_ */
