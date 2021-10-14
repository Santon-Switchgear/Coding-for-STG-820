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
#include "CANopen.h"
#include "stdlib.h"

typedef enum
{
  eTmr_LED,
  eTmr_Reset,
	eTmr_Calibration,
	eTmr_CanSend,
	eTmr_Jumper,
  SW_TIMER_CNT
} eSwTimer_t;

/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t au16Timer[SW_TIMER_CNT];
__IO uint8_t u8TmrCallbackEnabled = 0;
uint16_t u16Voltage = 0;
uint8_t u8I = 0;
bool Calibration=0;
float Calibrationcounter0=2;
float Calibrationcounter1=2;
float Calibrationcounter2=2;
float Calibrationcounter3=2;
float CalibratedMIN=0;
float CalibratedMAX=0;
float limitcounter = 0;
bool jumper = 0;
bool factory_reset = 0;
uint16_t MINtemp;
uint16_t MAXtemp;
bool write_to_failstate_memory = 0;

/* EEPROM declared Private variables ---------------------------------------------------------*/
uint8_t u8WrSetup;//Setup_Complete
uint8_t u8WrSetupOld;//Setup_Complete_Old
uint8_t u8RdSetup;
uint8_t FAILSTATE = false;
uint8_t FAILSTATEold;
//uint8_t MAX1 = 12;//11; //15;//4030 separated in 2x 8 bit numbers
//uint8_t MAX2 = 48;//236; //190;
uint16_t MAX=1188;
//uint8_t MIN1 = 15;//11;//3052
//uint8_t MIN2 = 220;//190; //236;
uint16_t MIN=204;//225;
//uint8_t MAX1old;//4030 separated in 2x 8 bit numbers
//uint8_t MAX2old;
uint16_t MAXold;
//uint8_t MIN1old;//3052
//uint8_t MIN2old;
uint16_t MINold;
uint8_t Calibrated = 0;
bool jumperold;


//DEBUG variables
float MAXTEST;
float MINTEST;

/* CANOPEN declared Private variables ---------------------------------------------------------*/
typedef	union
{
	uint32_t u32[2];
	uint16_t u16[4];
	uint8_t u8[8];
	int32_t i32[2];
	int16_t i16[4];
	int8_t i8[8];
} CAN_Variant_t;

CAN_Variant_t CanMSG;

#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

/* Global variables and objects */
    volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */ 
		volatile bool Mode = false;
		volatile float Encoder_Set = 300; // Encoder = position 0
		volatile float CAN_DATA[10];//CANDATA array for data validation
		
/* Private function prototypes -----------------------------------------------*/
uint8_t u8Data[32];


/* timer thread executes in constant intervals ********************************/
static void tmrTask_thread(void)
{
  //for(;;) 
	{
    /* sleep for interval */

    INCREMENT_1MS(CO_timer1ms);

    if(CO->CANmodule[0]->CANnormal) 
		{
			bool_t syncWas;

			/* Process Sync and read inputs */
			syncWas = CO_process_SYNC_RPDO(CO, TMR_TASK_INTERVAL);

			/* Further I/O or nonblocking application code may go here. */

			/* Write outputs */
			CO_process_TPDO(CO, syncWas, TMR_TASK_INTERVAL);

			/* verify timer overflow */
			if(0) 
			{
					CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
			}
    }
  }
}


/* CAN interrupt function *****************************************************/
void /* interrupt */ CO_CAN1InterruptHandler(void)
{
  CO_CANinterrupt_Rx(CO->CANmodule[0]);


  /* clear interrupt flag */
} 


/* XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */

void vReset ( void )
{

}

void vCalibration ( void )
{
	// jumper ___|---|___|---|___|---|___|
	// state     1   2   3   4	 5	 6		7
	//Event				Calibration^	       		^FACTORY RESET
  static uint8_t u8State = 0;
	
	switch ( u8State )
	{
		case 0: // waiting for "Calibration triggered When in Reset/EMERGENCY position of controller"
			if ( true /*"Calibration triggered When in Reset/EMERGENCY position of controller"*/ )
				u8State++;
			break;
		case 1: // waiting for start " Jumper =0x01 for duration of 2 seconds"
			if ( jumper )
			{ // ____|------> 1. raise edge
			  au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				u8State++;
			}
			break;
		case 2: // waiting for end " Jumper =0x01 for duration of 2 seconds"
			if ( !jumper )
			{ // ------|_____> 1. falling edge 
			  if ( au16Timer[eTmr_Calibration] == 0 ) // more as 2000 ms = 2s
				{
					// start waiting 2 s for Jumper = 0
					au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				  u8State++;
				}
				else
					u8State = 0; // sequence not completetd
			}
			break;
		case 3: // waiting for start " Jumper =0x01 for duration of 2 seconds"
			if ( jumper )
			{ // ____|------> 2. raise edge
			  if ( au16Timer[eTmr_Calibration] == 0 ) // more as 2000 ms = 2s
				{
					// start waiting 2 s for Jumper = 1
					au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				  u8State++;
				}
				else
					u8State = 0; // sequence not completetd
			}
			break;
		case 4: // waiting for end " Jumper =0x01 for duration of 2 seconds"
			if ( !jumper )
			{ // ------|_____> 2. falling edge 
			  if ( au16Timer[eTmr_Calibration] == 0 ) // more as 2000 ms = 2s
				{
					// start waiting 2 s for Jumper = 0
					au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				  Calibration = 0x01;
					Calibrated = false;
					CalibratedMIN = false;
					CalibratedMAX = false;
					//Calibration_protocol();
					u8State++;
					
				}
				else
					u8State = 0; // sequence not completetd
			}
			break;
		case 5:
			if ( jumper )
			{ // ____|------> 2. raise edge
			  if ( au16Timer[eTmr_Calibration] == 0 ) // more as 2000 ms = 2s
				{
					// start waiting 2 s for Jumper = 1
					au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				  u8State++;
				}
				else
					u8State = 0; // sequence not completetd
			}
      
		break;
		case 6:
			if ( !jumper )
			{ // ____|------> 2. raise edge
			  if ( au16Timer[eTmr_Calibration] == 0 ) // more as 2000 ms = 2s
				{
					// start waiting 2 s for Jumper = 1
					au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				  u8State++;
				}
				else
					u8State = 0; // sequence not completetd
			}
      
		break;
		case 7:
			if ( jumper && (HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin)) )//FACTORY RESET VARIABLES & EEPROM
			{ // ____|------> 2. raise edge
			  if ( au16Timer[eTmr_Calibration] == 0  ) // more as 2000 ms = 2s
				{
					// start waiting 5 s for Jumper = 1
					au16Timer[eTmr_Calibration] = 2000; // 2000 ms = 2s
				  uint8_t u8Temp, u8I;
						//uint16_t u16a = 0x5555;
						u8Temp = 0xFF;
						
						if ( u8Temp == 0xFF )
						{
							// first init of EEPROM
							u8Temp = 0;
							for ( u8I=0; u8I<20; u8I++ )
								EEPROM_Write((uint16_t)u8I, &u8Temp , 1);
						}
						
						FAILSTATE = false;
						//Calibrated = false;
						FAILSTATEold=false;
						
						u8Temp=0;
						for ( u8I=0; u8I<10; u8I++)
						{
							CAN_DATA[u8I] = 0;
						}
						MAX=3120;
						MIN=4060;
						u8State = 0;
						HAL_NVIC_SystemReset();
				}
				else
					u8State = 0; // sequence not completetd
			}
			else
//				if ( au16Timer[eTmr_Calibration] == 0  )
//				{
//					u8State = 0;
//				}
		break;
	}
}

/* XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */


/**
  * @brief  Transmission  complete callback in non blocking mode 
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	// Implement CAN RX
	CO_CAN1InterruptHandler();
	
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
}

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
	uint8_t u8I;
	// Decrement u16Timer every 1 ms down to 0
	for (u8I=0; u8I<SW_TIMER_CNT; u8I++ )
		if ( au16Timer[u8I] > 0 )
			au16Timer[u8I]--;
	
	if ( u8TmrCallbackEnabled )
	  tmrTask_thread();
}


		

bool SW1(){//s1 analog to bool conversion with threshhold 20mV
		float IN2 = ReadAnalogInput(ADC_IN2);
	  bool Sw1 = 0;
		if(IN2 < 20)
			{
				Sw1=0;
			} 
		else 
			{
				Sw1=1;
			}
		
		return !Sw1;
}

bool SW2(){//s2 analog to bool conversion with threshhold 20mV
		float IN3 = ReadAnalogInput(ADC_IN3);
	  bool Sw2 = 0;
		if(IN3 < 20)
			{
				Sw2=0;
			} 
		else 
			{
				Sw2=1;
			}
		
		return !Sw2;
}

bool SW3(){//s2 analog to bool conversion with threshhold 20mV
		
	  bool Sw3 = HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin);
	
		return !Sw3;
}
bool SW4(){//s2 analog to bool conversion with threshhold 20mV
		
	  bool Sw4 = HAL_GPIO_ReadPin(DIN5_Port,DIN5_Pin);
	
		return !Sw4;
}

void vReadJumper ( void )
{
	static uint8_t u8Laststate = 0;
	uint8_t u8State = 0;
	
	if ( au16Timer[eTmr_Jumper] == 0 )
	{
		au16Timer[eTmr_Jumper] = 5;
		u8State = HAL_GPIO_ReadPin(DIN6_Port,DIN6_HS_Pin);
		if ( u8Laststate == u8State )
			jumper = u8State;
    u8Laststate = u8State;
	}
}

 
int Initialize_outputs(){
	//STG-826 
	// 6 INPUTS(3X DIGITAL,3 X ANALOG 0-34VDC), 
	// 4 OUTPUTS (3X DIGITAL, 1X ANALOG)
	//========================================================================

	//initialize Analog output 5V //for encoder with DAC functionality (OUT4) analogset to 5V
	SetAnalogOutput(5100);
	//Initialize Jumper output set HIGH (24V)
	HAL_GPIO_WritePin(Out1_HS_GPIO_Port, Out1_HS_Pin, GPIO_PIN_SET); //JUMPER Set pin 2 (OUT1) HIGH / TRUE
	
	//Initialize Microswitch Status powered outputs
	HAL_GPIO_WritePin(Out2_HS_GPIO_Port, Out2_HS_Pin, GPIO_PIN_SET); //MICROSWITCH 1 + 2 Set pin 3 (OUT2) HIGH / TRUE
	HAL_GPIO_WritePin(Out3_HS_GPIO_Port, Out3_HS_Pin, GPIO_PIN_SET); //MICROSWITCH 3 + 4 Set pin 4 (OUT3) HIGH / TRUE
	

	return(true);
}


float EN1_filter()//uint16_t n)
{
		/** OLD code
	
	
	Enc_Val_raw = ReadAnalogInput(ADC_IN1);
	
	//n=1;
	//uint16_t data[n];
	float SUM = 0;
	float Enc_val = 0;
	float Enc_Val_raw;
	float Average;
	int i = 1;
	bool state=1;
//	while (i<=n)
//		{
//		  uint8_t Enc_old; 
//			Enc_Val_raw = ReadAnalogInput(ADC_IN1);//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);// READ ENCODER VALUE float value NOT digital value
//			//Normalization
//			//____________________
//			
//			
//			//data[n] = Enc_val;
////			if(state) {Enc_old = Enc_Val_raw; state =0; }
////			else {state=1;}
////			//Enc_Val_raw = 90*Enc_Val_raw+(1-90)*Enc_old;//exponential filter
//			SUM = SUM + Enc_Val_raw;   							//Sum for Average 
//			i = i+1;														//value counter
//			
//		}
	Average = (long)SUM/(long)(n);
	//SUM
	Average = Average - 300;//Subtraction
	Average = Average / 970;      			//Division
	Average = Average * 1023;           //GAIN
	//HAL_Delay(100);
		*/
	
	
	
	
	float Enc_Val_raw = ReadAnalogInput(ADC_IN1);
	
	if (!Calibrated) 
		{
			MAX = MAX;//MAX;
			MIN = MIN;//MIN;
		}
	else
		{
				MAX = MAXold; //(MAX1old << 8 ) | (MAX2old & 0xff);
				MIN = MINold; //(MIN1old << 8 ) | (MIN2old & 0xff);
		}
	MAXTEST = MAX;
	MINTEST= MIN;
	float Enc_Val =(((Enc_Val_raw-MIN)/(MAX-MIN))*1023);//1023-(((Enc_Val_raw-293)/962)*1023);//300)/910)*1023);//-308)/962)*1023);//(((Enc_Val_raw-285)/918)*1023);


	if ( Enc_Val < 30)//EMG
		{
			Enc_Val = 0;
		}
	if ( Enc_Val >= 1014)//TMAX
		{
			Enc_Val = 1023;
		}
	if ( 570 > Enc_Val && Enc_Val > 530)//IDLE
		{
			Enc_Val = 546;
		}
	if ( 152 > Enc_Val && Enc_Val > 114)//BMAX
		{
			Enc_Val = 114;
		}
	if ( 455 > Enc_Val && Enc_Val > 440)//BMIN
		{
			Enc_Val = 455;
		}		
	if ( 640 > Enc_Val && Enc_Val > 635)//TMIN
		{
			Enc_Val = 636;
		}
		
	return((long)Enc_Val);//Enc_Val_raw);//
}

int Calibration_protocol()
	 {
		 
		 //HAL_Delay(5);
		 //EEPROM_Read(0x0010, &Calibrated, 1);
		 

		 
		 if (Calibrated == 0x00)
			 {
//				 	EEPROM_Read(0x0003, &MAX1old, 1 );
//					EEPROM_Read(0x0004, &MAX2old, 1 );
//					EEPROM_Read(0x0005, &MIN1old, 1);
//					EEPROM_Read(0x0006, &MIN2old, 1);
				 EEPROM_Read(0x0003, (uint8_t*)&MAXold, 2 );
				 EEPROM_Read(0x0005, (uint8_t*)&MINold, 2 );
//				  MAXold = (MAX1old << 8 ) | (MAX2old & 0xff);
//					MINold = (MIN1old << 8 ) | (MIN2old & 0xff);
				 
				 if(MAXold != MAX || MINold != MIN)
				 {
					 EEPROM_Write(0x0003, (uint8_t*)&MAX, 2);
//					 EEPROM_Write(0x0004, &MAX2, 1 );
					 EEPROM_Write(0x0005, (uint8_t*)&MIN, 2);
//					 EEPROM_Write(0x0006, &MIN2, 1);
					 HAL_Delay(50);
				 }
				 if (Calibration)
				 {
						
					 if (CalibratedMIN==0x00 && jumper && !SW3())//(CalibratedMIN==0x00 && jumper && SW3())
						 {
							 //MIN= ReadAnalogInput(ADC_IN1);
							 
							 uint16_t MINtemp =ReadAnalogInput(ADC_IN1);
							 MIN = MINtemp;
							 EEPROM_Write(0x0005,(uint8_t*)&MINtemp, 2 );
							 HAL_Delay(50);
							 CalibratedMIN=0x01;
							 
						 }
						 

						if (!jumper && CalibratedMIN ==0x01)
							{
							if (CalibratedMAX==0x00 && (SW3()))//(CalibratedMAX==0x00 && (!SW3()))
						  {
								uint16_t MAXtemp = ReadAnalogInput(ADC_IN1);
								
								EEPROM_Write(0x0003,(uint8_t*)&MAXtemp, 2 );
								HAL_Delay(50);
								uint8_t Calibrated_temp= 1;
								EEPROM_Write(0x0010,&Calibrated_temp , 1);
								FAILSTATE = 0;
								FAILSTATEold = 0;
								EEPROM_Write(0x0001, &FAILSTATE, 1);
								HAL_Delay(50);
								HAL_NVIC_SystemReset();
								
							}
							
								 
									
								
						}
					 
				 }
			 }
			if(Calibrated)
				{
					
					//HAL_Delay(10);

					MAX = MAXold;//(MAX1old << 8 ) | (MAX2old & 0xff);
					MIN = MINold;//(MIN1old << 8 ) | (MIN2old & 0xff);
					
					MAXTEST=MAX;
					MINTEST=MIN;
				}

		
		 
	 }
	 
bool FACTORYRESET()
{
	{
		uint8_t u8Temp, u8I;
		//uint16_t u16a = 0x5555;
		u8Temp = 0xFF;
		
	
		u8Temp = 0;
		for ( u8I=0; u8I<20; u8I++ )
			 EEPROM_Write((uint16_t)u8I, &u8Temp , 1);
		
		FAILSTATE = false;
		//Calibrated = false;
		FAILSTATEold=false;
		
		u8Temp=0;
		for ( u8I=0; u8I<10; u8I++)
		{
			CAN_DATA[u8I] = 0;
		}
		MAX=3120;
		MIN=4060;

	}
}

 float Validaton(){
	 
	 float Enc_valid = EN1_filter();
	 //CAN_DATA;
	 
	 if(Enc_valid > -1 && Enc_valid < 1024) // Datavalidility check {Enc_DataVal}
	 {
		 CAN_DATA[0] = true;
	 }
	 else
	 {
		 CAN_DATA[0] = false;
	 }
	 
	 if(Enc_valid >= 636 && Enc_valid <= 1023) // TRACTION Pos active {TrBr_T}
	 {
		 CAN_DATA[1] = true;
		 bool SW_1 = SW1();
		 if(!SW_1 && Enc_valid >= 680 && 1023 >= Enc_valid )//(SW_1 && Enc_valid >= 680 && 1023 >= Enc_valid )
			{
				CAN_DATA[5] = 1;//Check status of S1 {MICRO1_TrBr_Ko}
			}
		 if(!SW_1 && Enc_valid >= 670 && 1023 >= Enc_valid )//Check status of S1 {MICRO1_TrBr_Ko}
			{
				//CAN_DATA[5] = 0;
			}
	 }

	 else
	 {
		 CAN_DATA[1] = false;
		 bool SW_1 = SW1();
		
		 if(SW_1 && Enc_valid >= 0 && 602 >= Enc_valid)
			{
				//CAN_DATA[5] = 1;
			}
			if(SW_1 && Enc_valid >= 0 && 546 >= Enc_valid)//(!SW_1 && Enc_valid >= 0 && 546 >= Enc_valid)
			{
				CAN_DATA[5] = 1;//PLC1
			}
		 
	 }
	

	 if(Enc_valid > 455 && Enc_valid < 635) // IDLE Pos active {TrBr_Zero}
			{
				CAN_DATA[2] = true;
				bool SW_2 = SW2();
		 if(SW_2 && Enc_valid > 530 && Enc_valid < 603)//Check status of S2 {MICRO2_TrBr_Ko}
				{
						//CAN_DATA[6] = 0;
					
				}
		 if(SW_2 && Enc_valid > 535 && Enc_valid < 550)//(!SW_2 && Enc_valid > 535 && Enc_valid < 550)
				{
						CAN_DATA[6] = 1;//Check status of S2 {MICRO2_TrBr_Ko}
					bool F3=1;
				}
			}
	 else
		 {
			 
			 CAN_DATA[2] = false;
			 bool SW_2 = SW2();
			 bool F1=0;
			 bool F2=0;
			 if(!SW_2 && Enc_valid >= 0 && Enc_valid < 410)//(SW_2 && Enc_valid >= 0 && Enc_valid < 410)
					{
						CAN_DATA[6]=1;
						F1=1;
					}
			 if (!SW_2 && Enc_valid > 690 && Enc_valid < 1023)//(SW_2 && Enc_valid > 690 && Enc_valid < 1023)
					{
						CAN_DATA[6]=1;
						F2=1;
					}
			 if(!F1 && !F2)
				 {
					 //CAN_DATA[6] = 0;
				 }
		 }
	 
   if(Enc_valid >= 0 && Enc_valid < 500) // BRAKE Pos active {TrBr_B} STG-826
	 {
		 CAN_DATA[3] = true;
		 if(!SW4()&& Enc_valid < 365)//(SW4()&& Enc_valid < 400)
				{
					CAN_DATA[8] = 1;
				}
			else
				{
					//CAN_DATA[8]=;
				} //Check status of S4 {MICRO4_TrBr_Ko}
	 }
	 else
	 {
		 CAN_DATA[3] = false;
		 if(SW4()&& Enc_valid >= 546)
				{
					CAN_DATA[8] = 1;
				}
	 }
	 
	 
	 if(Enc_valid >= 0 && Enc_valid <= 100) // EMERGENCY Pos active {TrBr_EMG} S3 {MICRO3_TrBr_Ko}
	 {
		 CAN_DATA[4] = true;
		 if(SW3() && Enc_valid >= 0 && Enc_valid <= 40)
				{
					//CAN_DATA[7] = 0;
				}
			if((SW3()) &&  Enc_valid <= 30)//(!(SW3()) &&  Enc_valid <= 30)
				{
					CAN_DATA[7] = 1;
				}
			
	 }
	 else
	 {
		  CAN_DATA[4] = false;
			if(!(SW3()) && Enc_valid >= 148 && Enc_valid <= 1023)//((SW3()) && Enc_valid >= 148 && Enc_valid <= 1023)
				{
					CAN_DATA[7] = 2;
				}
		  
	 }

	 if(((!CAN_DATA[0]) || CAN_DATA[5] || CAN_DATA[6] || CAN_DATA[7] || CAN_DATA[8] || FAILSTATE) )
		{
			CAN_DATA[9] = 0;
			FAILSTATE = true;
		} 
		else
		{
			CAN_DATA[9]=1;
		}
	
		
	 
 }


 
 uint8_t Dataset(bool b1,bool b2,bool b3,bool b4,bool b5,bool b6,bool b7,bool b8)
{
	int i;

	static uint8_t dataBits[8]= {0};
	dataBits[0] = b1;
	dataBits[1] = b2;
	dataBits[2] = b3;
	dataBits[3] = b4;
	dataBits[4] = b5;
	dataBits[5] = b6;
	dataBits[6] = b7;
	dataBits[7] = b8;
	
//	//int value=0;
	uint8_t u8 = 0;
	int power = 1;
	
	for (i=0;i<8;i++)
	{
		u8 += dataBits[7-i]*power;
		power *=2;
	}
	
	
	for (i=0;i<2;i++)
	{
		u8 += dataBits[1-i]*power;
		power *=2;
	}

		
		
/*	
//	u8 |= ((uint8_t)b1 << 0);
//	u8 |= ((uint8_t)b2 << 1);
//	u8 |= ((uint8_t)b3 << 2);
//	u8 |= ((uint8_t)b4 << 3);
//	u8 |= ((uint8_t)b5 << 4);
//	u8 |= ((uint8_t)b6 << 5);
//	u8 |= ((uint8_t)b7 << 6);
//	u8 |= ((uint8_t)b8 << 7);*/
	
  return u8;
}
 


int main(void)
{
	// Do not change the system clock above 16 MHz! Higher speed can lead to the destruction of the module!
	
	
	//Can open reset init()
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT; 
	
  // System init	
  MainInit();
	
	HAL_Delay(100);
	{
		uint8_t u8Temp, u8I;
		//uint16_t u16a = 0x5555;
		EEPROM_Read(0x0000, &u8Temp, 1);

	  if ( u8Temp == 0xFF )
		{
		  // first init of EEPROM
			u8Temp = 0;
			for ( u8I=0; u8I<20; u8I++ )
			  EEPROM_Write((uint16_t)u8I, &u8Temp , 1);
		}
		EEPROM_Read(0x0010, &Calibrated, 1);
		EEPROM_Read(0x0001, &FAILSTATEold, 1);
		EEPROM_Read(0x0003, (uint8_t*)&MAXold, 2 );
		EEPROM_Read(0x0005, (uint8_t*)&MINold, 2 );
		
		
//		EEPROM_Write(0x0003, (uint8_t*)&u16a, 2 );
//		EEPROM_Read(0x0003, (uint8_t*)&u16a, 2 );
//		u16a = u16a;
	}
	uint8_t Start= 0;
	uint8_t test =0;
	//EEPROM_Write(0x00010,&test , 1);
	
	//EEPROM definitions
	//EEPROM_Read(0x0000, &u8RdSetup, 1); //Read value from EEPROM and store it in "u8Rd"
	//EEPROM_Read(0x0001, &FAILSTATEold, 1);
	u8WrSetup = u8RdSetup; //Set u8Wr to value of u8Rd after a reset, read and written value are identical
	u8WrSetupOld = u8RdSetup; //Set u8WrOld = u8Rd, so that no value will be written until u8Wr changes
	
	// Start DAC output
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	
	//Initialize Digital and Analog OUTPUTS as HIGH or 5V/5000mV
	bool STAT1 = Initialize_outputs();
	
	// LED On
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	
	// =======================================================================
	// Set up baudrate
	hcan.Instance->BTR &= 0xFFFFFC00;
	hcan.Instance->BTR |= CAN_250K;
	// =======================================================================
	
	/* increase powerOnCounter variable each startup. Variable is stored in EEPROM. */
	//OD_powerOnCounter++;

  /* Infinite loop */
	
  while (1)
  {
		// =======================================================================
		// CANOpen sample code
		// Untested port of https://github.com/CANopenNode/CANopenNode
		// =======================================================================
		
		while(reset != CO_RESET_APP)
		{
			/* CANopen communication reset - initialize CANopen objects *******************/
			CO_ReturnError_t err;
			uint16_t timer1msPrevious;
			
			/* disable CAN and CAN interrupts */
      CanDisable();
			uint8_t NodeID1 = 56;//57; //Default set Node ID if Jumper open/FALSE
			bool NodeID_condition = 0;
			
			vReadJumper();

			if (jumper)//ReadAnalogInput(ADC_IN2))  //Condition for noe ID is HIGH / TRUE
			{
				uint8_t NodeID1 = 58;//59; //CPU1-CAB2
				NodeID_condition = 1;
			}
			else
			{
				uint8_t NodeID1 = 56;//57; //CPU1-CAB1
				NodeID_condition = 0;
			}
			/* initialize CANopen */
			err = CO_init(0/* CAN module address */, NodeID1/* NodeID */, CAN_250K /* bit rate */);
			if(1)//err != CO_ERROR_NO)
			{
					while(1)
					{
    				vReadJumper();
						//int NodeID1 = 56; //Default set Node ID if Jumper open/FALSE
						bool NodeID = 0;
						if (jumper)//ReadAnalogInput(ADC_IN2))  //Condition is TRUE if pin 13 of register c is HIGH / TRUE
						{
							int NodeID1 = 58;//59;//CPU2_CAB2//58; //CPU1-CAB2
							NodeID_condition = 1;
						}
						else
						{
							int NodeID1 = 56;//57;//CPU2_CAB1////56; //CPU1-CAB1
							NodeID_condition = 0;
						}
						
						
						STAT1 = Initialize_outputs();

						
						// LED flicker for error
						if ( au16Timer[eTmr_LED] == 0 )
						{
							au16Timer[eTmr_LED] = 100;
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
						}


			/* Configure Timer interrupt function for execution every 1 millisecond */

			
			/* Configure CAN transmit and receive interrupt */
      CanEnable();


			/* start CAN */
			CO_CANsetNormalMode(CO->CANmodule[0]);

			reset = CO_RESET_NOT;
			timer1msPrevious = CO_timer1ms;
			
			// Enable timer
			u8TmrCallbackEnabled = 1;

			while(reset == CO_RESET_NOT )//|| 1)
			{
				/* loop for normal program execution ******************************************/
				uint16_t timer1msCopy, timer1msDiff;

				timer1msCopy = CO_timer1ms;
				timer1msDiff = timer1msCopy - timer1msPrevious;
				timer1msPrevious = timer1msCopy;

				vReadJumper();
				
				vCalibration();
				
				/* CANopen process */
			//	reset = CO_process(CO, timer1msDiff, NULL);

				/* Nonblocking application code may go here. */
				// LED handling:
				STAT1 = Initialize_outputs();
				
				// Send it by CAN
				hcan.pTxMsg->IDE = CAN_ID_STD;

				if (jumper)//)
					{
						hcan.pTxMsg->StdId = 0x00003A;//0x00003B;   //Reciever adres: 0x003A (DMA-15)
					}
				else
					{
						hcan.pTxMsg->StdId = 0x000038;//  0x000039; //Reciever adres: 0x0038 (DMA-15)
					} 
				
				hcan.pTxMsg->DLC = 4 ;					
				
				//Debugging code-----------------------------
			//_____________________________________________
				
				float JumperState = HAL_GPIO_ReadPin(DIN6_Port,DIN6_HS_Pin);//ReadAnalogInput(ADC_IN2);
				
				float EncoderState = ReadAnalogInput(ADC_IN1);
				float EncoderState21 =	EN1_filter();
				Encoder_Set = EncoderState+0;
				float EncoderState3 = (((EncoderState-300)/910)*1023);
				float EncoderState4 = (((EncoderState-308)/962)*1023);
				bool S1 = SW1();
			  bool S2 = SW2();//ReadAnalogInput(ADC_IN3);//s2
				bool S3 = SW3();//s3
				bool Jumpers = jumper;
				bool S4 = SW4();
				
			//---------------------------------------------
			//---------------------------------------------
				//HAL_Delay(1);	
				EEPROM_Read(0x0000, &u8WrSetup, 1); //Read value from EEPROM and store it in "u8Rd"
				EEPROM_Read(0x0001, &FAILSTATEold, 1);
				if (factory_reset)
					{
						FACTORYRESET();
					}
				
				
				
				if ( u8WrSetup == 0) // Write data to EEPROM if not run ( One time run )
				{
					u8WrSetup = 1;//Set Setup to 0x01
					FAILSTATEold = 0x00;//Set FAILSTATE to 0x00
					EEPROM_Write(0x0000, &u8WrSetup, 1);//Set Setup to 0x01 and write to EEPROM
					//EEPROM_Write(0x0001, &FAILSTATEold, 1);//Set FAILSTATE to 0x00 and write to EEPROM
//					EEPROM_Write(0x0003, &MAX1, 1 );
//					EEPROM_Write(0x0004, &MAX2, 1 );
					EEPROM_Write(0x0003, (uint8_t*)&MAX, 2 );
//					EEPROM_Write(0x0005, &MIN1, 1);
//					EEPROM_Write(0x0006, &MIN2, 1);
					EEPROM_Write(0x0005, (uint8_t*)&MIN, 2 );
					//Calibration_protocol();
					HAL_Delay(1000);
					HAL_NVIC_SystemReset();
				}
				if ( FAILSTATE != FAILSTATEold) // Write data to EEPROM if changed
				{
					FAILSTATE = FAILSTATEold;
					//EEPROM_Write(0x0001, &FAILSTATE, 1);
					//HAL_Delay(50);
				}
				
				float Enc_Val_filtered = EN1_filter();//Readout sensor value 0.21-4.08V translate to 0-1023 and filter noise for n variables
				
				if (Calibration)
				{
					Enc_Val_filtered = ReadAnalogInput(ADC_IN1);
				}
				if (Start > 2)
				{
					Validaton();//Function to validate the microswitches and encoder validility and convert them to an array
				}
				
				if (Start<3)
				{
					Start++;
				}
				if ( FAILSTATE != FAILSTATEold) // Write data to EEPROM if changed
				{
					FAILSTATEold = FAILSTATE;
					write_to_failstate_memory = 1;

				}
				
				long Enc_Val_filtered1 = (long)Enc_Val_filtered;

				CanMSG.u32[0] = 0;
				CanMSG.u32[1] = 0;
				
				CanMSG.u8[0] = *((uint8_t*)&(Enc_Val_filtered1)+1); //high byte (0x12)Enc_Val_filtered;
				CanMSG.u8[1] = *((uint8_t*)&(Enc_Val_filtered1)+0); //low byte  (0x34)Enc_Val_filtered;
				bool Enc_Data_Val 	= (bool)CAN_DATA[0];//129   10000001
				bool TrBr_T 				= (bool)CAN_DATA[1];
				bool TrBr_Zero			= (bool)CAN_DATA[2];//163   10100011
				bool TrBr_B					= (bool)CAN_DATA[3];//197   11000101
				bool TrBr_EMG				= (bool)CAN_DATA[4];// 1    00000001
				bool MICRO1_TrBr_Ko	= (bool)CAN_DATA[5];
				bool MICRO2_TrBr_Ko	= (bool)CAN_DATA[6];
				bool MICRO3_TrBr_Ko	= (bool)CAN_DATA[7];//129   10000001
				bool MICRO4_TrBr_Ko	= (bool)CAN_DATA[8];
				bool TrBr_dataValid = (bool)CAN_DATA[9];	
				
				uint8_t dataset1 = Dataset(CAN_DATA[0],CAN_DATA[1],CAN_DATA[2],CAN_DATA[3],CAN_DATA[4],CAN_DATA[5],CAN_DATA[6],CAN_DATA[7]);
				uint8_t dataset2 = Dataset(0,0,0,0,0,Calibration,CAN_DATA[8],CAN_DATA[9]);
				CanMSG.u8[2] = dataset1;
				CanMSG.u8[3] = dataset2;
				// Transfer data
				hcan.pTxMsg->Data[0] = CanMSG.u8[0];
				hcan.pTxMsg->Data[1] = CanMSG.u8[1];
				hcan.pTxMsg->Data[2] = CanMSG.u8[2];
				hcan.pTxMsg->Data[3] = CanMSG.u8[3];
				
				
//				for ( u8I=0; u8I<8; u8I++ )
//					hcan.pTxMsg->Data[u8I] = CanMSG.u8[u8I];
				// Send it by CAN
				
				if ( au16Timer[eTmr_CanSend] == 0 )
				{
				  HAL_CAN_Transmit(&hcan, 25); //XXXXXXXX you are sending the CAN message all the time - is this right???? Have you PCAN for monitoring????
					au16Timer[eTmr_CanSend] = 25;
				}
				
				{

				
					if ( au16Timer[eTmr_LED] == 0 )
					{
						au16Timer[eTmr_LED] = 1000;
						HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
						EEPROM_Read(0x0000, &u8WrSetup, 1); //Read value from EEPROM and store it in "u8Rd"
				    EEPROM_Read(0x0001, &FAILSTATEold, 1);
						if (write_to_failstate_memory)
							{
								EEPROM_Write(0x0001, &FAILSTATE, 1);
								HAL_Delay(50);
								write_to_failstate_memory = 0;
							}
						Calibration_protocol();
						
						if (jumper && HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin) && Calibrationcounter0 == 2 && Calibrationcounter2 == 2 )
							{
								Calibrationcounter1--;
							}
						if (!(jumper) && (HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin)) && Calibrationcounter1 == 0 && Calibrationcounter2 == 2)//JUMPER IN5 -> IN6
							{
								Calibrationcounter0--;
							}
						if (jumper && Calibrationcounter0 == 0 && Calibrationcounter1 == 0 && Calibrationcounter3 == 2 )
							{
								Calibrationcounter2--;							
							}
						if (!jumper && Calibrationcounter0 == 0 && Calibrationcounter1 == 0 && Calibrationcounter2 == 0)
							{
								Calibrationcounter3--;
								if (Calibrationcounter3 ==0)
									{
										//Calibration= true;
										//Calibrated = false;
										//CalibratedMIN = false;
									//	CalibratedMAX = false;
									}
							}
						if (Calibrationcounter0 < -1 ||Calibrationcounter1 < -1 ||Calibrationcounter1 < -1 ||Calibrationcounter2 < -1)
							{
								Calibrationcounter0 = 2;
								Calibrationcounter1 = 2;
								Calibrationcounter2 = 2;
								Calibrationcounter3 = 2;
								
							}
						if (Calibrationcounter0 != 2 || Calibrationcounter1 != 2 || Calibrationcounter2 != 2 || Calibrationcounter3 != 2)
							{
								limitcounter++;
								if (limitcounter >30)
									{
										Calibrationcounter0 = 2;
										Calibrationcounter1 = 2;
										Calibrationcounter2 = 2;
										Calibrationcounter3 = 2;
									}
							}
						Calibration_protocol();
					}
				}
								/* CANopen process */
				reset = CO_process(CO, timer1msDiff, NULL);
				// Watchdog refresh
				#if ( PRODUCTION_VERSION == 1 )
					HAL_IWDG_Refresh(&hiwdg);
					#warning Production version, Debugging not possible! <<<<<<<<<<<<<<<<<<<<<
				#else
					#warning Debug version without watch dog! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				#endif

				/* Process EEPROM */
			}
		}			

    /* stop threads */

    /* delete objects from memory */
    CO_delete(0/* CAN module address */);
 		
		
  }
}
}}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
