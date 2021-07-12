/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define CAN_1M 1
#define CAN_500K 2
#define CAN_250K 4
#define CAN_125K 8
#define CAN_100K 10
#define CAN_50K 20
#define ADC_IN1 1
#define ADC_IN2 2
#define ADC_IN3 3

#define Out1_HS_Pin GPIO_PIN_13
#define Out1_HS_GPIO_Port GPIOC
#define Out2_HS_Pin GPIO_PIN_14
#define Out2_HS_GPIO_Port GPIOC
#define Out3_HS_Pin GPIO_PIN_15
#define Out3_HS_GPIO_Port GPIOC
#define Out4_HS_Pin GPIO_PIN_4
#define Out4_HS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define CAN_S_Pin GPIO_PIN_6
#define CAN_S_GPIO_Port GPIOB
#define Out5_LS_T17_1N_Pin GPIO_PIN_7
#define Out5_LS_T17_1N_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define PRODUCTION_VERSION 0 // Version for Debugging without Watch dog
//#define PRODUCTION_VERSION 1 // Version for Production with Watch dog

#define PG65_PARAMETER_COUNT 6 // Parameter count for PG-65 communication (1..100)

#define DIN4_Pin GPIO_PIN_12
#define DIN4_Port GPIOA
#define DIN5_Pin GPIO_PIN_15
#define DIN5_Port GPIOA

#define EEPROM_ADDRESS			0xA0
#define EEPROM_BUFFER_SIZE	32  // Page size
#define EEPROM_WRITE_TIME		5   // Page write time in ms
#define EEPROM_TIMEOUT			10  // timeout for wirite

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
