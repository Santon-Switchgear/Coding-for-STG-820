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
uint8_t u8Data[16];

/**
  * @brief  Transmission  complete callback in non blocking mode 
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	// Implement CAN RX
	if ( hcan->pRxMsg->IDE == CAN_ID_STD )
	{
		switch ( hcan->pRxMsg->StdId )
		{
			case 0x100:
				// Test received data an set OUT1
				if ( hcan->pRxMsg->Data[0] > 5 )
					HAL_GPIO_WritePin(Out1_HS_GPIO_Port, Out1_HS_Pin, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(Out1_HS_GPIO_Port, Out1_HS_Pin, GPIO_PIN_RESET);
				break;
		}
	}
  else
	{
		switch ( hcan->pRxMsg->ExtId )
		{
			
		}
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
}

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
	
	// =======================================================================
	// Set up baudrate
	hcan.Instance->BTR &= 0xFFFFFC00;
	hcan.Instance->BTR |= CAN_250K;
	// =======================================================================
	
	
  /* Infinite loop */
  while (1)
  {
		// =======================================================================
		// CAN sample code
		// Voltage at IN1 (V) will be send by the CAN (ID=0x3FF, DLC=1, Value in Byte 0)
		// if received value from CAN (ID=0x100, DLC=1, Value in Byte 0) greather 
		// than 5, the OUT1 will be Hight otherwise Low 
		// =======================================================================

		uint8_t u8Tx;

		// Read analog value from IN1 [V]
		u8Tx = ReadAnalogInput(ADC_IN1) / 1000;
		// Send it by CAN
		hcan.pTxMsg->IDE = CAN_ID_STD;
		hcan.pTxMsg->StdId = 0x3FF;
		hcan.pTxMsg->DLC = 1;
		hcan.pTxMsg->Data[0] = u8Tx;
		HAL_CAN_Transmit(&hcan, 10);

    // For receiving see the interrupt handler function HAL_CAN_RxCpltCallback


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
