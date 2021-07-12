#ifndef __MAIN_HAL_H
#define __MAIN_HAL_H

#include "main.h"
#include "stm32f0xx_hal.h"

typedef	__packed union
{
	uint8_t u8;
	int32_t i32;
	char  ch[5];
} CAN_PG65_Variant_t;

typedef	__packed struct
{
	uint16_t u16PId;
	uint8_t u8PNo;
	CAN_PG65_Variant_t v;
} CAN_PG65_t;


typedef struct
{
	int32_t i32LowerLimit;			// Minimal value
	int32_t i32UpperLimit;			// Maximal value
	int32_t i32Default;					// Default value
	int32_t i32StepSize;				// Step size for touch event
	uint8_t u8Decimals;					// Dezimal point: 0=1, 1=0.1, 2=0.01, 3=0.001
	uint8_t u8Unit;							// 0=NONE, 1=%, °C/°K/l/ml/m/mm/kg/g/10=kN/N/V/mV/A/mA/kOhm/Ohm/kW/W/20=h/min/s/ms/kHz/25=Hz
	char		cDescription[10];		// Descripion of parameter, less than 10 characters ends with 0x00(\N)
	uint8_t u8Function;					// 0=Read/Write, 1=ReadOnly
	int32_t i32Value;						// Actual value, used in your application
} PG65_t;

extern PG65_t PG65[PG65_PARAMETER_COUNT];


extern CanTxMsgTypeDef CAN_TX_Msg;
extern CanRxMsgTypeDef CAN_RX_Msg;
extern CanTxMsgTypeDef CAN_TX_Msg;
extern CanRxMsgTypeDef CAN_RX_Msg;
extern ADC_HandleTypeDef hadc;
extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;


void PG65Handler(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef EEPROM_Read(uint16_t BaseAddress, uint8_t* Data, uint16_t Size);
HAL_StatusTypeDef EEPROM_Write(uint16_t BaseAddress, uint8_t* Data, uint16_t Size);
uint16_t ReadAnalogInput (uint8_t Channel);
void DIn4ResetCounter ( void );
uint16_t DIn4ReadCounter ( void );
uint16_t DIn5ReadFrequency ( void );
uint16_t DIn5ReadDutyCycle ( void );
void MainInit ( void );

#endif /* __MAIN_HAL_H */
