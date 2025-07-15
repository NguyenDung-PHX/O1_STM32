/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#define Feedback_Hall 0x000001B65

#define VESC1_ID 0x40
#define VESC2_ID 0x43
#define VESC3_ID 0x65
#define VESC4_ID 0x64

#define MSG_TYPE_HALL 0x1B
#define MSG_TYPE_STATUS1 0x09
#define MSG_TYPE_STATUS2 0x02
#define MSG_TYPE_STATUS3 0x0F
#define MSG_TYPE_STATUS4 0x0E
#define MSG_TYPE_STATUS5 0x10

#define CAN_ID(motor, type)   ((type << 8) | (motor))

typedef struct {
	uint8_t data[8];
}Data8_Receiver_t;

typedef struct {
	int PPM;
	float Current;
	float Duty_Cucle;
}Status1_t;

typedef struct {
	float Ah_Used;
	float Ah_Charged;
}Status2_t;

typedef struct {
	float Wh_Used;
	float Wh_Charged;
}Status3_t;

typedef struct {
	int16_t Temp_FET;
	int16_t Temp_Motor;
	int16_t Current_In;
	int16_t PID_position_Now;
}Status4_t;

typedef struct {
	int16_t Voltage_In;
	int16_t Tachometer;
}Status5_t;

typedef struct {
	int Encoder;
	int16_t Receive;
}Status6_t; // hall

typedef struct {
	Status1_t Status1;
	Status2_t Status2;
	Status3_t Status3;
	Status4_t Status4;
	Status5_t Status5;
	Status6_t Status6;
	Data8_Receiver_t Data_Receiver;
}VESC_t;

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

