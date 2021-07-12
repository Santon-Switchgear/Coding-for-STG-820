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
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 0;
__IO uint16_t u16Timer1 = 0;
uint8_t u8First = 1;
uint16_t u16Tx = 0;

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
			case 0x7FC:
				// CAN receiving
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
  if ( u16Timer1 > 0 )
		u16Timer1--;
}

#define WAIT1 u16Timer1=10;while(u16Timer1);

void ShowTemplate1 ( void )
{
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;
	// Send it by CAN
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->StdId = 0x7FB;
	hcan.pTxMsg->DLC = 8;
	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 1;       // Template 2 (1V_BAR)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1;    // Variable 0 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Curre", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 2;    // Variable 1 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "nt   ", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 3; // Variable 2 (Parameter)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 22;      // 
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 4; // Variable 3 (Value)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx/10;   // 
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5; // Variable 4 (Unit)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 15;      // V
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 6; // Button 0 (Esc)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x0;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 7; // Button 1 (Up)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 8; // Button 2 (Down)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 9; // Button 3 (OK)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); 
	}
}

void ShowTemplate2 ( void )
{
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;
	// Send it by CAN
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->StdId = 0x7FB;
	hcan.pTxMsg->DLC = 8;
	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 2;       // Template 2 (1V_BAR)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1;    // Variable 0 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Volta", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 2;    // Variable 1 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ge   ", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 3; // Variable 2 (Parameter)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 11;      // 
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 4; // Variable 3 (Value)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx/10;   // 
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5; // Variable 4 (Unit)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 12;      // V
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 6; // Variable 5 (Bargraph)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx/300;// %
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 7; // Button 0 (Esc)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x0;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 8; // Button 1 (Up)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 9; // Button 2 (Down)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 10; // Button 3 (OK)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x0;   // Visible
		HAL_CAN_Transmit(&hcan, 10); 
	}
}

void ShowTemplate3 ( void )
{
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;
	// Send it by CAN
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->StdId = 0x7FB;
	hcan.pTxMsg->DLC = 8;
	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 3;       // Template 2 (1V_BAR)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10);

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1;    // Variable 0 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Volta", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 2;    // Variable 1 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ge   ", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 3;    // Variable 0 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Curre", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 4;    // Variable 1 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "nt   ", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5;    // Variable 0 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Press", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 6;    // Variable 1 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ure  ", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 7;    // Variable 0 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Force", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1

		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 8;    // Variable 1 (Text)
		strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
		(*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 9; // (Value)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx;   // 
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 10; // (Unit)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 12;      // V
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 11; // (Value)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx/11;   // 
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 12; // (Unit)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 15;      // mA
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 13; // (Value)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx/13;   // 
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 14; // (Unit)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 21;      // N
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00;   // Visible
		HAL_CAN_Transmit(&hcan, 10); WAIT1
	}
	
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 15; // (Value)
	(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = u16Tx/15;   // 
	(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
	HAL_CAN_Transmit(&hcan, 10); WAIT1

	if ( u8First )
	{
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 16; // (Unit)
		(*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 11;      // N
		(*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;   // Visible
		HAL_CAN_Transmit(&hcan, 10);
	}
}

uint8_t u8Flag = 0;
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

		if ( u8Flag )
		{
		  static uint8_t u8Template = 0;
			static uint8_t u8Cnt = 0;
			
			u8Flag = 0;

			if ( ++u8Cnt > 15 )
			{
			  u8Cnt = 0;
				u8Template++;
				if ( u8Template > 2 )
					u8Template = 0;
				u8First = 1;
			}
			
			//u8Template = 0; // uncomment and set to 0..2 for static template
			
			switch ( u8Template )
			{
				case 0: 
					ShowTemplate1();
          break;
				case 1: 
					ShowTemplate2();
					break;
				case 2: 
					ShowTemplate3();
					break;
			}
			
      u8First = 0;

		}

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
				u8Flag = 1;
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
