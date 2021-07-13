/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body Changed in accordance with 50Mz Project
	* Company						 : Santon Switchgear / santon Holland BV
	* Author/Editor			 : Zander van der Steege
	* Project code based on Example of STMicroelectronics
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
	MAIN.C + MAIN_HAL.C  and few others HAVE BEEN EDITED TO TRY GET PROJECT WORKING,
	THE FOLLOWING CHANGES ARE NOT WORKING YET, I WILL TRY FINING A SOLUTION 
	BUT	ALSO REQUEST BARTH ELEKTRONIK FOR ASSITANCE.
	_________________________________________
	1. CAN COMMUNICATION EXCEEDS 8 BYTE DATA 
	HOW DO I ASSIGN LARGER DATA TO THE CAN ID TRANSMISSION DATA? 16 BIT/32 BIT
	
	1 LONG, 10 BOOL VARIABLES
	_________________________________________
	2.ANALOG OUTPUT FUNCTION NOT WORKING DUE TO INCOORECLY IMPORTING INTO PROJECT
	IS THERE A MANUAL THAT I MISSED FOR THIS, CAN YOU FIX THE LINK FOR ME IN THE MEANTIME?
	
	_________________________________________
	3.
	I/O STG-820			I/O µC		GIPO_PIN
				IN 1			PA0
				IN 2			PA1
				IN 3			PA2
				IN 4			PA12
				IN 5			PA15
				IN 6			PA16?			
				OUT 1			PC13			GPIO_PIN_1
				OUT 2			PC14			GPIO_PIN_2
				OUT 3			PC15			GPIO_PIN_3
				OUT 4			PA4				GPIO_PIN_2
				OUT 5			PB7				NA
				IRDA_TX		PB10
				IRDA_RX		PB11
				RS232_TX	PA9
				RS232_RX	PA10
				CAN_TX		PB9
				CAN_RX		PB8
				LED	PA8

	
  */
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"                  // Device header
//#include "CANopen.h"

/* USER CODE BEGIN Includes */
#include "main_hal.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//CanTxMsgTypeDef CAN_TX_Msg;
//CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 0;
__IO uint8_t u8TmrCallbackEnabled = 0;
int NodeID = 56;

#define TMR_TASK_INTERVAL   (100)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t u16Voltage = 0;
/* USER CODE END 0 */








int main(void)
{

  /* USER CODE BEGIN 1 */
	// Do not change the system clock above 16 MHz! Higher speed can lead to the destruction of the module!
	
  // System init	
  MainInit();
	// LED On
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* USER CODE BEGIN 2 */
  
	// Start DAC output
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// =======================================================================
		// Analog Output sample code
		// Increment Output voltage every one ms by 50 mV
		// If Output voltage overstepped 5100 mV it starts with 0
		// =======================================================================
		
		
		if (u16SwTimer[T2] == 0)
		{ 
			u16SwTimer[T2] = 1;
			
			u16Voltage = u16Voltage + 50;
			if ( u16Voltage >= 5100 )
				u16Voltage = 0;
			SetAnalogOutput(u16Voltage);
		}
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		if (u16SwTimer[LED] == 0)
		{ 
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			u16SwTimer[LED] = 1000;
		}
		
		
		// Watchdog refresh
		#if ( PRODUCTION_VERSION == 1 )
		  HAL_IWDG_Refresh(&hiwdg);
			#warning Production version, Debugging not possible! <<<<<<<<<<<<<<<<<<<<<
		#else
			#warning Debug version without watch dog! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		#endif
		
		
		
  }
  /* USER CODE END 3 */

}


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
