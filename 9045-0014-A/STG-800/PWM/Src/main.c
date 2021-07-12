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


int main(void)
{
	// Do not change the system clock above 16 MHz! Higher speed can lead to the destruction of the module!
	
  // System init	
  MainInit();
	// LED On
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	
  /* Infinite loop */
  while (1)
  {
		// =======================================================================
		// PWM calculation:
		// PSC [0..65535]: Prescaler (htim17.Instance->PSC)
		// ARR [0..65535]: Auto reload register (htim17.Instance->ARR), 
		//                 should be >99 for DC in % or >999 for DC in 1/10%
		// CCR1 [0..65535]: Capture compare register (htim17.Instance->CCR1)
		// Function: 16 MHz / (PSC+1) / (ARR+1) -> Frequency [Hz]
		//           100 * (CCR1+1) / (ARR+1) -> Duty Cycle [%]
		//
		// ARR = 100..1000..65535 - 1
		// PSC = (16000000 / (fOut * (ARR+1))) - 1
		// CCR1 = (DC[%] * (ARR+1) / 100%) - 1
		// =======================================================================

		
		// =======================================================================
		// PWM sample code
		// Depending from voltage on DIN4 the frequency of DOUT5 (Low Side) will be changed
		// =======================================================================
		
		if ( HAL_GPIO_ReadPin(DIN4_Port, DIN4_Pin) == GPIO_PIN_SET )
		{
			// Input active -> set frequency to 100 Hz and DC to 75 %
			
			// ARR = 100-1 (DC in %), check: 0 <= ARR <= 65535 - value OK
			htim17.Instance->ARR = 100-1;
			// PSC = (16000000 / (100 Hz * (99 + 1))) - 1 = 1599, check: 0 <= PSC <= 65535 - value OK
			htim17.Instance->PSC = 1599;
			// CCR1 = (75 * (99+1) / 100) - 1 = 74, check: 0 <= CCR1 <= 65535 - value OK
			htim17.Instance->CCR1 = 74;
		}
		else
		{
			// Input low -> set frequency to 200 Hz and DC to 25 %
			
			// ARR = 100-1 (DC in %), check: 0 <= ARR <= 65535 - value OK
			htim17.Instance->ARR = 100-1;
			// PSC = (16000000 / (200 Hz * (99 + 1))) - 1 = 799, check: 0 <= PSC <= 65535 - value OK
			htim17.Instance->PSC = 799;
			// CCR1 = (25 * (99+1) / 100) - 1 = 24, check: 0 <= CCR1 <= 65535 - value OK
			htim17.Instance->CCR1 = 24;
		}
		
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
