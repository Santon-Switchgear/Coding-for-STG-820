/**test
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body Changed in accordance with 50Mz Project
	* Company						 : Santon Switchgear / santon Holland BV
	* Author/Editor			 : Zander van der Steege
	* Project code based on Example of STMicroelectronics
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
//#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_dac_ex.h"
//#include "main_hal.h"
#include "CANopen.h"
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 0;
__IO uint8_t u8TmrCallbackEnabled = 0;
int NodeID = 56;

#define TMR_TASK_INTERVAL   (100)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

/* Global variables and objects */
    volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */  //UNSURE OF PURPOSE
		volatile bool FAILSTATE = false; // added foas global FAILSTATE variable

/* Private function prototypes -----------------------------------------------*/ //UNSURE OF PURPOSE
uint8_t u8Data[16];


/* timer thread executes in constant intervals ********************************/ //UNSURE OF PURPOSE
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
	// Decrement u16Timer every 1 ms down to 0
  if ( u16Timer > 0 )
		u16Timer--;
	
	if ( u8TmrCallbackEnabled )
	  tmrTask_thread();
}


int Initialize_outputs(){
	//STG-826 
	// 6 INPUTS(3X DIGITAL,3 X ANALOG 0-34VDC), 
	// 4 OUTPUTS (3X DIGITAL, 1X ANALOG)
	//========================================================================
	
	
	//initialize Analog output 5V 
	//for encoder with DAC functionality (OUT4) analogset to 5V
	//DAC1_CHANNEL_1_WritePin(GPIOC,GPIO_PIN_4,50); // trigger set analog OUT4 to 5V
	
//	SetAnalogOutput(5100);
	//Initialize Jumper output set HIGH (24V)
	HAL_GPIO_WritePin(Out1_HS_GPIO_Port, Out1_HS_Pin, GPIO_PIN_SET); //JUMPER Set pin 2 (OUT1) HIGH / TRUE
	
	//Initialize Microswitch Status powered outputs
	HAL_GPIO_WritePin(Out2_HS_GPIO_Port, Out2_HS_Pin, GPIO_PIN_SET); //MICROSWITCH 1 + 2 Set pin 3 (OUT2) HIGH / TRUE
	HAL_GPIO_WritePin(Out3_HS_GPIO_Port, Out3_HS_Pin, GPIO_PIN_SET); //MICROSWITCH 3 + 4 Set pin 4 (OUT3) HIGH / TRUE
	

	return(true);
}


float EN1_filter(n)
{
	
	//uint16_t data[n];
	float SUM = 0;
	float Enc_val = 0;
	int N=n;
	long Average;
	while (n<=25)
		{
		float Enc_Val_raw = ReadAnalogInput(ADC_IN1);//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);// READ ENCODER VALUE float value NOT digital value
		//Normalization
		//____________________
		
		Enc_val = Enc_Val_raw - 2.917;//Subtraction
		Enc_val = Enc_val / 0.94;      			//Division
		Enc_val = Enc_val * 1023;           //GAIN
	  //data[n] = Enc_val;
		SUM = SUM + Enc_val;   							//Sum for Average 
		n = n+1;														//value counter
		}
	
	return(Average = (long)SUM/(long)N);
}

 int * Validaton(Enc_valid){
	 static int Data[10]= {0};
	 
	 if(Enc_valid > -5 && Enc_valid < 1028) // Datavalidility check {Enc_DataVal}
	 {
		 Data[0] = true;
	 }
	 else
	 {
		 Data[0] = false;
	 }
	 
	 if(Enc_valid > 631 && Enc_valid < 1028) // TRACTION Pos active {TrBr_T}
	 {
		 Data[1] = true;
		 if(ReadAnalogInput(ADC_IN3)>10){Data[5] = 0;}else{Data[5]=1;} //Check status of S1 {MICRO1_TrBr_Ko}
	 }
	 else
	 {
		 Data[1] = false;
		 if(ReadAnalogInput(ADC_IN3)>10){Data[5] = 1;}else{Data[5]=0;}
	 }
	 
	 if(Enc_valid > 541 && Enc_valid < 551) // IDLE Pos active {TrBr_Zero}
	 {
		 Data[2] = true;
		 if(HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin)){Data[6] = 0;}else{Data[6]=1;} //Check status of S2 {MICRO2_TrBr_Ko}
	 }
	 else
	 {
		 Data[2] = false;
		 if(HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin)){Data[6] = 1;}else{Data[6]=0;}
	 }
	 
//	 if(Enc_valid > 109 && Enc_valid < 500) // BRAKE Pos active {TrBr_B}
//	 {
//		 Data[3] = true;
//		 if(HAL_GPIO_ReadPin(DIN6_Port,DIN6_Pin)){Data[8] = 0;}else{Data[8]=1;} //Check status of S4 {MICRO4_TrBr_Ko}
//	 }
//	 else
//	 {
//		 Data[3] = false;
//		 if(HAL_GPIO_ReadPin(DIN6_Port,DIN6_Pin)){Data[8] = 1;}else{Data[8]=0;}
//	 }
	 
	 	 if(Enc_valid > -5 && Enc_valid < 5) // EMERGENCY Pos active {TrBr_EMG} 
	 {
		 Data[4] = true;
		 if(HAL_GPIO_ReadPin(DIN5_Port,DIN5_Pin)){Data[7] = 1;}else{Data[7]=0;} //Check status of S3 {MICRO3_TrBr_Ko}
	 }
	 else
	 {
		 Data[4] = false;
		 if(HAL_GPIO_ReadPin(DIN5_Port,DIN5_Pin)){Data[7] = 0;}else{Data[7]=1;}
	 }
	 if( Data[5] || Data[6] || Data[7] || Data[8] || FAILSTATE)
		{
			Data[9] = 0;
			FAILSTATE = true;
		} 
		else
		{
			Data[9]=1;
		}
	 return(Data);
 }


 int Dataset(bool b1,bool b2,bool b3,bool b4,bool b5,bool b6,bool b7,bool b8)
{
	static int dataBits[8]= {0};
	int value=0;
	
	dataBits[0] = b1;
	dataBits[1] = b2; 
	dataBits[2] = b3;
	dataBits[3] = b4;
	dataBits[4] = b5;
	dataBits[5] = b6;
	dataBits[6] = b7;
	dataBits[7] = b8;
	
	dataBits[0] = (int)dataBits[0] << 0;
	dataBits[1] = (int)dataBits[0] << 1;
	dataBits[2] = (int)dataBits[0] << 2;
	dataBits[3] = (int)dataBits[0] << 3;
	dataBits[4] = (int)dataBits[0] << 4;
	dataBits[5] = (int)dataBits[0] << 5;
	dataBits[6] = (int)dataBits[0] << 6;
	dataBits[7] = (int)dataBits[0] << 7;
	
	for(int i = 0;i < sizeof(dataBits);i++)
	{
		value |= (int)dataBits[i] << i;
	}
	//value = (int)dataBits[0] << 7| (int)dataBits[0] << 6 | (int)dataBits[0] << 5 |(int)dataBits[0] << 4 |(int)dataBits[0] << 3 |(int)dataBits[0] << 2 |(int)dataBits[0] << 1| (int)dataBits[0] << 0;
	return(value);
		
}
 

int main(void)
{
	// Do not change the system clock above 16 MHz! Higher speed can lead to the destruction of the module!
	
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT; 
  // System init	
  MainInit();
	// LED On
	bool STAT2 = Initialize_outputs();
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	HAL_DAC_Init()
	// Setup Analog output										//ANALOG SETUP WITH DAC FILES ARE NOT LINKING UNSURE WHY FOR THE MOMENT
	//#include "stm32f0xx_hal_dac_ex.c"
	//#include "stm32f0xx_hal_dac_ex.h"
	//#include "stm32f0xx_hal_dac.c"
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);			// I TRIED ADDING DAC TO MAIN_HAL.C I MIGHT BE DOING THIS INCORRECTLY
																						// I NEED TO FIGURE OUT HOW TO INSERT NEW LIBRARY'S TO PROJECT EXAMPLE: DAC 
	SetAnalogOutput(5100);//5000 mV output constant (max 5100)
	// =======================================================================
	// Set up baudrate
	hcan.Instance->BTR &= 0xFFFFFC00;
	hcan.Instance->BTR |= CAN_250K;
	// =======================================================================
	
	/* increase variable each startup. Variable should be stored in EEPROM. */
	OD_powerOnCounter++;
 	
  /* Infinite loop */
  while (1)
  {
		// =======================================================================
		// CANOpen sample code
		// Untested port of https://github.com/CANopenNode/CANopenNode
		// =======================================================================
		bool STAT1 = Initialize_outputs(); // Initialization of outputs Analog and digital
		//========================================================================
			//CAN-Open NodeID depending on if Jumper of OUT1 is TRUE/FALSE
			//TRUE : closed/TRUE (Node ID = 58) 
			//FALSE: open/FALSE  (Node ID = 56)
			//========================================================================
			int NodeID1 = 56; //Default set Node ID if Jumper open/FALSE
			bool NodeID = 0;
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))  //Condition is TRUE if pin 13 of register c is HIGH / TRUE
			{
				int NodeID1 = 58; //CPU1-CAB2
				NodeID = 1;
			}
			else
			{
				int NodeID1 = 56; //CPU1-CAB1
				NodeID = 0;
			}
			
			
			STAT1 = Initialize_outputs();
			//========================================================================
			//Read analog inputs and call arithmitic function for n values of encoder
			//1. Moving average function + convert Float to LONG int 
			//2. Switch status readout
			//========================================================================
			 
			
			long Enc_Val_filtered = EN1_filter(25);//Readout sensor value 0.21-4.08V translate to 0-1023 and filter noise for n variables
			
			int * CAN_DATA[10] = {Validaton(Enc_Val_filtered)};//Function to validate the microswitches and encoder validility and convert them to an array

		while(reset != CO_RESET_APP)
		{
			
			
			
			
			/* CANopen communication reset - initialize CANopen objects *******************/
			CO_ReturnError_t err;
			uint16_t timer1msPrevious;

			/* disable CAN and CAN interrupts */
      CanDisable();

			/* initialize CANopen */
			err = CO_init(0/* CAN module address */, NodeID1 /* NodeID */, CAN_250K /* bit rate */);
			if(err != CO_ERROR_NO)
			{
					while(1)
					{
						// LED flicker for error
						if ( u16Timer == 0 )
						{
							u16Timer = 100;
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
						}
					}
					/* CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err); */
			}


			/* Configure Timer interrupt function for execution every 1 millisecond */
			
			//50MZ CANOPEN SETUP Not functioning
			//==========================================================================
			hcan.pTxMsg->IDE = CAN_ID_STD;//Standard-ID
			if (NodeID){hcan.pTxMsg->StdId = 0x003A;   //Reciever adres: 0x003A (DMA-15)
				}
			else{hcan.pTxMsg->StdId = 0x0038;   //Reciever adres: 0x0038 (DMA-15)
				}
			int * Data[16] = {0};
			
			hcan.pTxMsg->DLC = 18;         //Message length: 16 bytes (0-1023 long int + 10 bool binary variables
//			//==========================================================================
			/* increase variable each startup. Variable should be stored in EEPROM. 
			Enc_Val 				[long]		0x09		0-8		Byte 1
			
			Enc_Data_Val		[bool]		0x01		0			Byte 2
			TrBr_T 					[bool]		0x01		1			...	
			TrBr_Zero				[bool]		0x01		2			...
			TrBr_B     			[bool]		0x01		3			...
			TrBr_EMG				[bool]		0x01		4			...
			MICRO1_TrBr_Ko	[bool]		0x01		5			...
			MICRO2_TrBr_Ko	[bool]		0x01		6			...
			MICRO3_TrBr_Ko	[bool]		0x01		7			...
			
			MICRO4_TrBr_Ko	[bool]		0x01		0			Byte 3
			TrBr_dataValid	[bool]		0x01		1			...
			*/
			
//			//50MZ CANOPEN MSG Extracted from CAN_DATA list      !!!!!!NEED TO FIGURE OUT OUT TO INSERT CAN_DATA[] INTO Data[]
			hcan.pTxMsg-> Data [0] = 0x3FF; // Enc_Val 			[long]
			hcan.pTxMsg-> Data [1] = 0xFF; // dataset 1			[word]
			hcan.pTxMsg-> Data [2] = 0xFF; // dataset 2			[word]
			hcan.pTxMsg-> Data [3] = 0x03; // dataset 3			[word]
			hcan.pTxMsg-> Data [4] = 0x00; // 
			hcan.pTxMsg-> Data [5] = 0x00; // 
			hcan.pTxMsg-> Data [6] = 0x00; // 
			hcan.pTxMsg-> Data [7] = 0x00; // 
			HAL_CAN_Transmit (& hcan, 10);	HAL_Delay(10);  // 10ms time delay for safe transmission
//			}
			long Enc_Val = Enc_Val_filtered;
			bool Enc_Data_Val 	= CAN_DATA[0];
			bool TrBr_T 				= CAN_DATA[1];
			bool TrBr_Zero			= CAN_DATA[2];
			bool TrBr_B					= CAN_DATA[3];
			bool TrBr_EMG				= CAN_DATA[4];
			bool MICRO1_TrBr_Ko	= CAN_DATA[5];
			bool MICRO2_TrBr_Ko	= CAN_DATA[6];
			bool MICRO3_TrBr_Ko	= CAN_DATA[7];
			bool MICRO4_TrBr_Ko	= CAN_DATA[8];
			bool TrBr_dataValid = CAN_DATA[9];	
			
			int dataset1 = Dataset(Enc_Data_Val,TrBr_T,TrBr_Zero,TrBr_B,TrBr_EMG,MICRO1_TrBr_Ko,MICRO2_TrBr_Ko,MICRO3_TrBr_Ko);
			int dataset2 = Dataset(MICRO4_TrBr_Ko,TrBr_dataValid,0,0,0,0,0,0);
			
			hcan.pTxMsg->Data[0] = Enc_Val;
			hcan.pTxMsg->Data[1] = dataset1;
			hcan.pTxMsg->Data[2] = dataset2;
//			hcan.pTxMsg->Data[0] = Enc_Val; //Encoder Value 0-1023
//			hcan.pTxMsg->Data[1] = Enc_Data_Val;    //Verify Encoder is within range
//			hcan.pTxMsg->Data[2] = TrBr_T; //Enc_Val within Traction position range 
//			hcan.pTxMsg->Data[3] = TrBr_Zero; //Enc_Val within IDLE/Zero position range 
//			hcan.pTxMsg->Data[4] = TrBr_B; //Enc_Val within Brake position range 
//			hcan.pTxMsg->Data[5] = TrBr_EMG; //Enc_Val within Emergency position range 
//			hcan.pTxMsg->Data[6] = MICRO1_TrBr_Ko; //Micro switch not in error (=0/FALSE) (within range of TRACTION)
//			hcan.pTxMsg->Data[7] = MICRO2_TrBr_Ko; //Micro switch not in error (=0/FALSE) (within range of IDLE)
//			hcan.pTxMsg->Data[8] = MICRO3_TrBr_Ko; //Micro switch not in error (=0/FALSE) (within range of EMERGENCY)
//			hcan.pTxMsg->Data[9] = MICRO4_TrBr_Ko; //Micro switch not in error (=0/FALSE) (within range of BRAKE)
//			hcan.pTxMsg->Data[10] = TrBr_dataValid;//Micro switches 1-4 not in error (=0/FALSE) (within range)			
//			HAL_CAN_Transmit(&hcan, 10);  //10ms Zeitverzögerung für sicheres Senden	
			/* Configure CAN transmit and receive interrupt */
      CanEnable();


			/* start CAN */
			CO_CANsetNormalMode(CO->CANmodule[0]);

			reset = CO_RESET_NOT;
			timer1msPrevious = CO_timer1ms;
			
			// Enable timer
			u8TmrCallbackEnabled = 1;

			while(reset == CO_RESET_NOT)
			{
				/* loop for normal program execution ******************************************/
				uint16_t timer1msCopy, timer1msDiff;

				timer1msCopy = CO_timer1ms;
				timer1msDiff = timer1msCopy - timer1msPrevious;
				timer1msPrevious = timer1msCopy;


				/* CANopen process */
				reset = CO_process(CO, timer1msDiff, NULL);

				/* Nonblocking application code may go here. */
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

				/* Process EEPROM */
			}
		}			

    /* stop threads */

    /* delete objects from memory */
    CO_delete(NodeID1/* CAN module address */);
 		
		
  }
}


/************************ (C) COPYRIGHT STMicroelectronics & Santon Switchgear *****END OF FILE****/
