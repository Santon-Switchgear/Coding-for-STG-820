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

// Parameter table for PG-65, see file main.h
//	int32_t i32LowerLimit;			// Minimal value
//	int32_t i32UpperLimit;			// Maximal value
//	int32_t i32Default;					// Default value
//	int32_t i32StepSize;				// Step size for touch event
//	uint8_t u8Decimals;					// Dezimal point: 0=1, 1=0.1, 2=0.01, 3=0.001
//	uint8_t u8Unit;							// 0=NONE, 1=%, °C/°K/l/ml/m/mm/kg/g/10=kN/N/V/mV/A/mA/kOhm/Ohm/kW/W/20=h/min/s/ms/kHz/25=Hz
//	char		cDescription[10];		// Descripion of parameter, less than 10 characters ends with 0x00(\N)
//  uint8_t u8Function;					// 0=Read/Write, 1=ReadOnly
//	int32_t i32Value;						// Actual value, used in your application

PG65_t PG65[PG65_PARAMETER_COUNT] = {
// i32LowerLimit i32UpperLimit i32Default i32StepSize u8Decimals u8Unit cDescription u8Function i32Value
{		0,						100,					0,					5,					0,				0,			"Pressure  ",		0,				0},  // 0/5/10/.../100									PG65[0].i32Value
{		-1000,				1000,					0,					50,					1,				12,			"Voltage   ",		0,				0},  // -100.0/-95.0/-90.0/.../100.0 V	PG65[1].i32Value
{		-1000,				1000,			  	0,					50,		  		1,				15,			"Current   ",		0,				0},  // -100.0/-95.0/-90.0/.../100.0 mA	PG65[2].i32Value
{		0,						10000,				0,					500,				2,				11,			"Force     ",		0,				0},  // 0.00/5.00/10.00/.../100.00 N		PG65[3].i32Value
{		0,						1000,					0,					20,					1,				7,			"Height    ",		1,				0},  // 0.0/2.0/4.0/.../100.0 mm				PG65[4].i32Value
{		0,						1000,					0,					50,					1,				6,			"Altitude  ",		0,				0},  // 0.0/5.0/10.0/.../100.0 m				PG65[5].i32Value
};
// PG65_PARAMETER_COUNT must be exact the data set count!
// Parameter usage for application: PG65[4].i32Value = 0;


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
			case 0x7FE:
				PG65Handler(hcan);
			  break;
			case 0x100:
				// other CAN messages ...
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
		// PG-65 by CAN sample code
		// Parameters from the PG65 structure can be showed/changed by CAN
		// =======================================================================

    // For PG-65 handling see the interrupt handler function HAL_CAN_RxCpltCallback


    // LED handling:
		{
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
