/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "main_hal.h"

/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 0;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
	// Decrement u16Timer every 1 ms down to 0
  if ( u16Timer > 0 )
		u16Timer--;
}

uint16_t u16Frequency;
uint8_t u8DutyCycle;

int main(void)
{
	// Do not change the system clock above 16 MHz! Higher speed can lead to the destruction of the module!
	
  // System init	
  MainInit();
	// LED On
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	
	// =======================================================================
	// Setup and Functionality:
	// PSC = Prescaler (/1../65536)
	// CNT = Counter (0..65565)
	// 16 MHz -> /PSC  -> Gate -> CNT 
	// Input >--------------^
	// Sample: measure maximal frequency of 100 Hz with 1 % accuracy 
	// Requirement: 100 Hz with 1 % accuracy = 10000 Tics/s -> CNT = 10000/s -> fIN_CNT >= 10 kHz
	// Solution: 16 MHz / 10 kHz = 1600 -> PSC=1600-1
	// minimal frequency: CNTmax/10 kHz for overflow: 65535/10000 = 6,55 s = 0,153 Hz
	htim2.Instance->PSC = 1600-1;
	// Result calculation:
	// f = 10 kHz / htim2.Instance->CCR1
	// DC = htim2.Instance->CCR1 * 100 / htim2.Instance->CCR1 [%]
	// =======================================================================
	
	
  /* Infinite loop */
  while (1)
  {
		
		
		
		// =======================================================================
		// Frequency Counter sample code
		// PWM at IN5 will be measured for Frequency and Duty Cycle
		// Results stored in variables, visible by Debugger
		// =======================================================================
		
		
		u16Frequency = DIn5ReadFrequency(); // Hz
    u8DutyCycle = DIn5ReadDutyCycle(); // %
		
	
		// LED handling:
		{
			//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);	
			//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	
		
			if ( u16Timer == 0 )
			{
				u16Timer = 1000;
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			}
		}
		
		// Watchdog refresh
		#if ( PRODUCTION_VERSION == 1 )
		  HAL_IWDG_Refresh(&hiwdg);
			#warning Production version, Debugging not possible! <<<<<<<<<<<<<<<<<<<<<
		#else
			#warning Debug version without watch dog! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		#endif
		
  }
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
