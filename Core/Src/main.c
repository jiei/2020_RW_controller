/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lpme1_driver.h"
#include "lpme1.h"
#include "bldc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Enable printf
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/*
 * Mode current control or speed control
 */
#define CURRENT_MODE
//#define SPEED_MODE

/*
 * Enable FB
 */
#define ENABLE_FB

/*
 * Enable Beep
 */
//#define ENABLE_BEEP

#ifndef PI
#define PI 3.1415
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct
{
    float target_angle;
    float target_speed;
    float actual_angle;
    float actual_speed;
    float angle_base;
    float angle_error;
    float speed_error;
} Machine_Angle;

Lpme1_Data lpme1Data;
Machine_Angle ma;
float tmp_output=0;
float K_angle=0.011, K_omega=0.01;
float torque_coef = 75.76;
float max_current = 0.8;

uint8_t speed_observation_flag=0;
uint8_t observation_counter=0;
float now_speed=0, old_speed=0;

TIM_OC_InitTypeDef ConfigOC_rw;
Bldc_State rw_motor;

uint8_t logger_flag=0;
uint8_t print_flag=0;
int counter=0;
int counter_th=1000;	// 1000 / 50[Hz] = 20[s] logging
uint8_t uart_key=1;
uint8_t xbee_key_flag=0;
uint8_t key='0';

static uint32_t ADC_Data1[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void __io_putchar(uint8_t ch) {
	if(uart_key==1){
		HAL_UART_Transmit(&huart1, &ch, 1, 1);
	} else if(uart_key==2){
		HAL_UART_Transmit(&huart2, &ch, 1, 1);
	}
}
/*
PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
}
*/
void setup(void);
void loop(void);

void buzzer_output(_Bool state);
void motor_control(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void setup(void){
	_Bool ready_flag=1;
	//rw pwm pin setup
	ConfigOC_rw.OCMode = TIM_OCMODE_PWM1;
	ConfigOC_rw.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC_rw.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	ConfigOC_rw.OCFastMode = TIM_OCFAST_DISABLE;
	ConfigOC_rw.OCIdleState = TIM_OCIDLESTATE_RESET;
	ConfigOC_rw.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	ConfigOC_rw.Pulse=0;
	HAL_TIM_PWM_ConfigChannel(&htim1,&ConfigOC_rw,TIM_CHANNEL_1);
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	//ADC setup
	/*
	 * なぜかDMAのAD変換とfloat型のprintfが両立できない→ログ取りは基本int型でやる
	 */
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_Data1, sizeof(ADC_Data1)) != HAL_OK){
		 Error_Handler();
	}

	setbuf(stdout, NULL);

	//Motor variable initialization
	rw_motor.actual_current = 0;
	rw_motor.actual_speed = 0;
	rw_motor.target_current = 0;
	rw_motor.target_speed = 0;
	rw_motor.direction = 1;
	rw_motor.enable = 0;
	rw_motor.pwm_pulse = (htim1.Init.Period + 1) * 0.5;

	ma.target_angle = 0;
	ma.target_speed = 0;
	ma.actual_angle = 0;
	ma.actual_speed = 0;
	ma.angle_base = 0;


	HAL_Delay(300);
	while(1) {
		if(lpme1_set_Freq(LPMS_DATA_FREQ_200HZ) != LPME1_OK) ready_flag=0;
		if(lpme1_set_filter(LPMS_FILTER_MODE_MAD_GYR_ACC_MAG)!=LPME1_OK) ready_flag=0;
		if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) ready_flag=0;
		//if(HAL_UART_Receive_IT(&huart1, &key, 1) != HAL_OK) ready_flag=0;
		HAL_Delay(100);
		if(ready_flag == 1) break;
		ready_flag = 1;
	}
	uart_key=2;
	printf("Time,Angle Error,Speed Error,Actual Current,Wheel Speed\r\n");

	//Ready beep
	for(int i=0;i<3;i++){
		buzzer_output(1);
		HAL_Delay(50);
		buzzer_output(0);
		HAL_Delay(50);
	}
}
void loop(void){
	//update xbee command
	HAL_UART_Receive(&huart1,&key, 1, 10);

	switch(key){
	case 's':	//stop
		uart_key=1;
		printf("\r\nSTOP\r\n");
		rw_motor.enable=0;
		rw_motor.target_speed=0;
		rw_motor.target_current=0;
		break;
	case 'i':	//start
		uart_key=1;
		printf("\r\nCURRENT ENABLE\r\n");
		//rw_motor.enable=1;
		//rw_motor.target_speed=4000;
		rw_motor.target_current=0.3;
		rw_motor.enable=1;
		break;
	case 'o':
		uart_key=1;
		//printf("\r\nCURRENT ENABLE\r\n");
		rw_motor.target_current=-0.1;
		rw_motor.enable=1;
		break;
	case 'x':	//control start
		uart_key=1;
		printf("\r\nControl Start !!!\r\n");
		ma.angle_base = lpme1Data.euler[2];
		ma.target_angle = ma.angle_base + PI;
		ma.target_speed = 0;
		rw_motor.enable=1;
		logger_flag = 1;
		break;
	case 'X':
		uart_key=1;
		printf("\r\nControl Start !!!\r\n");
		/*ma.angle_base = lpme1Data.euler[2];
		ma.target_angle = ma.angle_base + PI;
		ma.target_speed = 0;
		rw_motor.enable=1;*/
		//logger_flag = 1;
		speed_observation_flag=1;
		break;

	case 'e':	//enable
		uart_key=1;
		printf("\r\nMOTOR ENABLE\r\n");
		rw_motor.enable=1;
		break;
	case 'p':	//change target speed
		uart_key=1;
		printf("\r\neneble:%d, direction:%d, speed:%d, current:%d\r\n", rw_motor.enable, rw_motor.direction,(int)(1000 * rw_motor.target_current), (int)(rw_motor.target_speed));
		break;
	case 'R':	//system reset
		HAL_NVIC_SystemReset();
		break;
	default:

		break;
	}
	key='0';

	uart_key=1;
	//printf("\r\n current:%d, speed:%d\r\n",(int)(1000 * rw_motor.actual_current), (int)(rw_motor.actual_speed));
	//printf("\r\nangle:%d, speed:%d target_current:%d\r\n", (int)(180*lpme1Data.euler[2]/PI), (int)(180*lpme1Data.gyr[1]/PI), (int)(tmp_output*1000));
	//printf("actual_angle:%d\r\n", (int)(180*ma.angle_error/PI));

	if(logger_flag==0 && counter>counter_th){
		buzzer_output(1);
		counter=0;
		HAL_Delay(1000);
		buzzer_output(0);
	}

	/*if(print_flag==1){
		uart_key=1;
		//printf("roll:%d, pitch:%d, yaw:%d\r\n", (int)(180*lpme1Data.euler[0]/PI), (int)(180*lpme1Data.euler[1]/PI), (int)(180*lpme1Data.euler[2]/PI));
	}*/

	HAL_Delay(500);
}

void buzzer_output(_Bool state){
#ifdef ENABLE_BEEP
	if(state==1){
		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
	}
#endif
}

void motor_control(void){
	rw_motor.actual_current = bldc_get_current(ADC_Data1[0]);
	rw_motor.actual_speed = bldc_get_speed(ADC_Data1[1]);

	ma.actual_angle = lpme1Data.euler[2] - ma.angle_base;
	if(ma.actual_angle < (-1)*PI){
		ma.actual_angle = (-1) * ma.actual_angle - PI;
	} else if(ma.actual_angle>PI){
		ma.actual_angle = (-1) * ma.actual_angle + PI;
	}

	//ベクトルの内積と外積を利用して目標角と現在角の偏差を計算する
	ma.angle_error = acos(cos(lpme1Data.euler[2]) * cos(ma.target_angle) + sin(lpme1Data.euler[2]) * sin(ma.target_angle));
	if((cos(lpme1Data.euler[2]) * sin(ma.target_angle) - sin(lpme1Data.euler[2]) * cos(ma.target_angle)) < 0){
		ma.angle_error = (-1) * ma.angle_error;
	}

	ma.speed_error = ma.target_speed - lpme1Data.gyr[1];

#ifdef ENABLE_FB
	tmp_output = torque_coef * (ma.angle_error * K_angle + ma.speed_error * K_omega);
	if(tmp_output > max_current){
		tmp_output = max_current;
	} else if(tmp_output < (-1)*max_current){
		tmp_output = (-1)*max_current;
	}
	rw_motor.target_current = tmp_output;
#endif

#ifdef CURRENT_MODE
	if(rw_motor.target_current < 0){
		rw_motor.direction = 1;
	} else {
		rw_motor.direction = 0;
	}
	rw_motor.pwm_pulse = bldc_current_to_pulse(rw_motor.target_current);
#endif

#ifdef SPEED_MODE
	if(rw_motor.target_speed > 0){
		rw_motor.direction = 1;
	} else {
		rw_motor.direction = 0;
	}
	rw_motor.pwm_pulse = bldc_speed_to_pulse(rw_motor.target_speed);
#endif

	if(rw_motor.enable==1){
		HAL_GPIO_WritePin(rw_enable_GPIO_Port, rw_enable_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(rw_enable_GPIO_Port, rw_enable_Pin, GPIO_PIN_RESET);
	}

	if(rw_motor.direction==1){
		HAL_GPIO_WritePin(GPIOB, rw_direction_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, rw_direction_Pin, GPIO_PIN_RESET);
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, rw_motor.pwm_pulse);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2){
		/* IMUセンサデータの更新 */
		if(lpme1_get_timestamp(&lpme1Data.time)==LPME1_OK) HAL_GPIO_TogglePin(GPIOB, user_led1_Pin);	//For instant visual debug
		//lpme1_get_acc(lpme1Data.acc);
		lpme1_get_gyr(lpme1Data.gyr);
		//lpme1_get_mag(lpme1Data.mag);
		lpme1_get_euler(lpme1Data.euler);
		//lpme1_get_quat(lpme1Data.quat);

		/* モータ制御値の計算，モータ関連ピンの制御 */
		motor_control();

		if(logger_flag==1){
			uart_key = 2;
			//Time [ms], ma.angle_error [*10^-2 deg], ma.speed_error [*10^-2 deg/s], actual_current [*10^-2 mA], actual_speed [rpm]
			printf("%d,%d,%d,%d,%d\r\n", (int)(1000 * lpme1Data.time), (int)(18000 * ma.angle_error / PI), (int)(18000 * ma.speed_error / PI), (int)(100000 * rw_motor.actual_current), (int)(rw_motor.actual_speed));
			//printf("%d,%d,%d\r\n", (int)(1000 * lpme1Data.time), (int)(18000 * lpme1Data.euler[2] / PI), (int)(18000 * lpme1Data.gyr[1] / PI));
			counter++;
		}
		if(counter>counter_th){
			logger_flag=0;
		}

		/* テザーのねじれ状態から安定状態へ遷移させる制御に使用
		 * 機体角速度が最大になる瞬間　=　釣り合いの姿勢角度
		 * とみなし，その瞬間の角度を目標として制御する．
		 * 実際の安定角度と機体角速度が最大になる瞬間には差が存在するため，
		 * ベストな推定法ではない
		 */

		if(speed_observation_flag==1){
			old_speed = now_speed;
			now_speed = lpme1Data.gyr[1];
			if(fabs(now_speed) < fabs(old_speed)){
				observation_counter++;
			} else {
				observation_counter=0;
			}
			if(observation_counter>5){
				ma.angle_base = lpme1Data.euler[2];
				ma.target_angle = ma.angle_base;
				ma.target_speed = 0;
				rw_motor.enable = 1;
				speed_observation_flag=0;
				logger_flag = 1;
			}
		}
	}
	HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &huart1){

		//xbee_key_flag=1;
	}
	HAL_UART_Receive_IT(&huart1, &key, 1);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_TogglePin(GPIOB, user_led1_Pin);
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
