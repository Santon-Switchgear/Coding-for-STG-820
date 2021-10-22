#ifndef __MAIN_HAL_H
#define __MAIN_HAL_H

#include "main.h"
#include "stm32f0xx_hal.h"

#define PRODUCTION_VERSION 0 // Version for Debugging without Watch dog
//#define PRODUCTION_VERSION 1 // Version for Production with Watch dog


#define EEPROM_ADDRESS			0xA0
#define EEPROM_BUFFER_SIZE	32  // Page size
#define EEPROM_WRITE_TIME		5   // Page write time in ms
#define EEPROM_TIMEOUT			10  // timeout for wirite


#define SW_TIMER_CNT 5
enum SWT {IrDA, SMTO, T2, LED, T4};
extern volatile uint16_t u16SwTimer[SW_TIMER_CNT];

extern ADC_HandleTypeDef hadc;
extern CAN_HandleTypeDef hcan;
extern DAC_HandleTypeDef hdac;
extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern IRDA_HandleTypeDef hirda3;

extern CanTxMsgTypeDef CAN_TX_Msg;
extern CanRxMsgTypeDef CAN_RX_Msg;

void SetAnalogOutput ( uint16_t u16mV );
HAL_StatusTypeDef EEPROM_Read(uint16_t BaseAddress, uint8_t* Data, uint16_t Size);
HAL_StatusTypeDef EEPROM_Write(uint16_t BaseAddress, uint8_t* Data, uint16_t Size);
uint16_t ReadAnalogInput (uint8_t Channel);
void DIn4ResetCounter ( void );
uint16_t DIn4ReadCounter ( void );
uint16_t DIn5ReadFrequency ( void );
uint16_t DIn5ReadDutyCycle ( void );
void MainInit ( void );

#endif /* __MAIN_HAL_H */
