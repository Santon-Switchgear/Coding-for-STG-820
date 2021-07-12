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

/** 
  * @brief  CAN Tx message structure definition  
  */
	
//typedef union
//{
//	uint32_t u32ID;
//	struct
//	{
//		uint32_t u8SA: 8;
//		uint32_t bi18PGN: 18;
//		uint32_t bi3Priority: 3;
//	} J1939ID_t;
//} J1939ID_t;

typedef union
{
	uint32_t u32ID;
	struct
	{
		uint32_t u8SA: 8;
		uint32_t bi21PGN: 21;
	} NMEA2000ID_t;
} NMEA2000ID_t;

typedef	union
{
	uint32_t u32[2];
	uint16_t u16[4];
	uint8_t u8[8];
	int32_t i32[2];
	int16_t i16[4];
	int8_t i8[8];
} CAN_Variant_t;


CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 0;
__IO uint8_t u8Flag = 0;

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
		// Send the ISO Acknowledgment (NMEA 2000) every one second
		// =======================================================================

		
		if ( u8Flag != 0 )
		{
			static NMEA2000ID_t NMEA2000ID;
			static CAN_Variant_t CanV;
			uint8_t u8I;
			
			u8Flag = 0; // Clear send flag
			
			// Content of ISO Acknowledgment (PGN 059392)
			uint8_t ControlByte = 2; // PGN supported but access denied
			uint8_t GroupFunctionValue = 0; // Not a group function PGN
			uint32_t PGNofReqestedInformation = 128267; // Water Depth
			
			// Store Content to the CAN message
			//  B0    B1    B2    B3    B4    B5    B6    B7
			// U8[0] U8[1] U8[2] U8[3] U8[4] U8[5] U8[6] U8[7]
			// <-U16[0]--> <-U16[1]--> <-U16[2]--> <-U16[3]-->
			// <-------U32[0]--------> <-------U32[1]-------->

			CanV.u8[0] = ControlByte;
			CanV.u8[1] = GroupFunctionValue;
			CanV.u8[2] = 0xFF; // Reserved bits set to 1
			CanV.u8[3] = 0xFF; // Reserved bits set to 1
			CanV.u8[4] = 0xFF; // Reserved bits set to 1
			CanV.u8[5] = (uint8_t)(PGNofReqestedInformation >> 0);
			CanV.u8[6] = (uint8_t)(PGNofReqestedInformation >> 8);
			CanV.u8[7] = (uint8_t)(PGNofReqestedInformation >> 16);
			
			// Fill CAN-ID
			NMEA2000ID.NMEA2000ID_t.bi21PGN = 59392;  // ISO Acknowledgment
			NMEA2000ID.NMEA2000ID_t.u8SA = 2;
			
			// Fill header 
			hcan.pTxMsg->IDE = CAN_ID_EXT;
			hcan.pTxMsg->ExtId = NMEA2000ID.u32ID;
			hcan.pTxMsg->DLC = 8;
			// Transfer data
			for ( u8I=0; u8I<8; u8I++ )
				hcan.pTxMsg->Data[u8I] = CanV.u8[u8I];
			// Send it by CAN
			HAL_CAN_Transmit(&hcan, 10);
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
