/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/*
//init UART module
uint8_t UART1_rxBuffer[12] = {0};
uint8_t Buffer[64] = {0};
uint8_t Text[12] = {0};
uint8_t length;

//init buffer for UART transmit of US-Sensor
uint8_t distance_str_front[200];
uint8_t len_front = 0;
uint8_t distance_str_left[200];
uint8_t len_left = 0;
uint8_t distance_str_right[200];
uint8_t len_right = 0;

//PLEASE ADD DESCRIPTION
uint8_t distance_str_od[200];
uint8_t len_od = 0;
*/

//define front US-Sensor corr. to US1
uint16_t echo_duration_front;
uint8_t echo_trig_front;
uint16_t t_0_front;
uint16_t t_1_front;

//define left US-Sensor corr. to US2
uint16_t echo_duration_left;
uint8_t echo_trig_left;
uint16_t t_0_left;
uint16_t t_1_left;

//define right US-Sensor corr. to US3
uint16_t echo_duration_right;
uint8_t echo_trig_right;
uint16_t t_0_right;
uint16_t t_1_right;

//define US selector [0,1,2]
uint8_t US_Select;

//PLEASE ADD DESCIPTION
uint8_t i;
uint16_t current_dis;
uint16_t ds;
uint16_t target_dis;
uint16_t p;
uint16_t sp;

uint16_t od;
uint16_t od_buf;

// send tm ds distance command trigger
uint8_t tmds_trig;

// send tm od distance command trigger
uint8_t tmod_trig;

//move distance command trigger
uint8_t mvds_trig; 
uint16_t dis_val;

//move speed command trigger
uint8_t set_sp_trig;

//move follow wall trigger
uint8_t set_wa_trig;

//move direction
uint8_t mv_direction;

//turning variables
uint8_t r; //lre_stepper iterrator for right motor when turning
uint8_t l; //lre_stepper iterrator for left motor when turning
uint16_t rotation; //select rotation distance in []
uint16_t cur_rotation; //current rotation distance in []
uint8_t mvleft_trig;
uint8_t mvright_trig;
uint8_t in_rot;

//parking variables
uint8_t parking_trig;
uint8_t front_wall_trig;
uint8_t us_counter;
uint8_t len_front_parking;
uint8_t distance_str_parking[200];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define Trig_US2_Pin GPIO_PIN_1
#define Trig_US2_GPIO_Port GPIOA
#define Trig_US3_Pin GPIO_PIN_2
#define Trig_US3_GPIO_Port GPIOA
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define US3_Test_Pin GPIO_PIN_12
#define US3_Test_GPIO_Port GPIOB
#define US3_Test_EXTI_IRQn EXTI4_15_IRQn
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define US2_Pin_Pin GPIO_PIN_7
#define US2_Pin_GPIO_Port GPIOC
#define US2_Pin_EXTI_IRQn EXTI4_15_IRQn
#define US1_Pin_Pin GPIO_PIN_8
#define US1_Pin_GPIO_Port GPIOC
#define US1_Pin_EXTI_IRQn EXTI4_15_IRQn
#define Trig_US1_Pin GPIO_PIN_9
#define Trig_US1_GPIO_Port GPIOC
#define Trig_US3_Test_Pin GPIO_PIN_8
#define Trig_US3_Test_GPIO_Port GPIOA
#define USBF4_DM_Pin GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define M2_4_Pin GPIO_PIN_11
#define M2_4_GPIO_Port GPIOC
#define M2_3_Pin GPIO_PIN_12
#define M2_3_GPIO_Port GPIOC
#define M2_2_Pin GPIO_PIN_2
#define M2_2_GPIO_Port GPIOD
#define M2_1_Pin GPIO_PIN_3
#define M2_1_GPIO_Port GPIOB
#define M1_1_Pin GPIO_PIN_4
#define M1_1_GPIO_Port GPIOB
#define M1_2_Pin GPIO_PIN_5
#define M1_2_GPIO_Port GPIOB
#define M1_3_Pin GPIO_PIN_6
#define M1_3_GPIO_Port GPIOB
#define M1_4_Pin GPIO_PIN_7
#define M1_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
