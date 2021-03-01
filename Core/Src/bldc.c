/*
 * bldc.c
 *
 *  Created on: Jan 25, 2021
 *      Author: jack
 */

#ifndef SRC_BLDC_C_
#define SRC_BLDC_C_

#include "tim.h"
#include "bldc.h"

//adc_data -> RW回転速度
float bldc_get_speed(uint16_t adc_data){
	return (float)(2.0 * ((float)adc_data/4096.0 - 0.5) * MAX_SPEED_RANGE);
}

//adc_data -> モータ電流値
float bldc_get_current(uint16_t adc_data){
	return (float)(2.0 * ((float)adc_data/4096.0 - 0.5) * MAX_CURRENT_RANGE);
}

//目標電流 -> pwm入力
uint16_t bldc_current_to_pulse(float target_current){
	return (uint16_t)((htim1.Init.Period + 1) * (0.1 + 0.8 * fabs(target_current) / MAX_CURRENT_RANGE));
}

//目標速度 -> pwm入力
uint16_t bldc_speed_to_pulse(float target_speed){
	return (uint16_t)((htim1.Init.Period + 1) * (0.1 + 0.8 * fabs(target_speed) / MAX_SPEED_RANGE));
}

#endif /* SRC_BLDC_C_ */
