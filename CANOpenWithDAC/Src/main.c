/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
	* Company						 : Santon Switchgear
	* Device						 : STG-826
	* Project						 : 50MZ - Berna Project
	* Author						 : Zander van der Steege
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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

/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 0;
__IO uint8_t u8TmrCallbackEnabled = 0;
uint16_t u16Voltage = 0;
uint8_t u8I = 0;

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
/* definitions------------------------------------------------------------------*/
#define TMR_TASK_INTERVAL   (1000)          /* Interval of tmrTask thread in microseconds */
#define INCREMENT_1MS(var)  (var++)         /* Increment 1ms variable in tmrTask */

/* Global variables and objects */
    volatile uint16_t   CO_timer1ms = 0U;   /* variable increments each millisecond */ 
		volatile bool FAILSTATE = false; // added foas global FAILSTATE variable
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
	/**STG-826 Details (pinout)
	// 6 INPUTS(3X DIGITAL,3 X ANALOG 0-34VDC), 
	// 4 OUTPUTS (3X DIGITAL, 1X ANALOG)
	//========================================================================
	//DAC1_CHANNEL_1_WritePin(GPIOC,GPIO_PIN_4,50); // trigger set analog OUT4 to 5V*/
	
	//for Rotary position sensor with DAC functionality (OUT4) analogset to 5V
	SetAnalogOutput(5100);//initialize Analog output 5V 
	
	//Initialize Jumper output set HIGH (24V)
	HAL_GPIO_WritePin(Out1_HS_GPIO_Port, Out1_HS_Pin, GPIO_PIN_SET); //JUMPER Set pin 2 (OUT1) HIGH / TRUE
	
	//Initialize Microswitch Status powered outputs
	HAL_GPIO_WritePin(Out2_HS_GPIO_Port, Out2_HS_Pin, GPIO_PIN_SET); //MICROSWITCH 1 + 2 Set pin 3 (OUT2) HIGH / TRUE
	HAL_GPIO_WritePin(Out3_HS_GPIO_Port, Out3_HS_Pin, GPIO_PIN_SET); //MICROSWITCH 3 + 4 Set pin 4 (OUT3) HIGH / TRUE
	

	return(true);
}


uint16_t EN1_filter(uint8_t n)
{
	
	//uint16_t data[n]; //data array for normalization
	float SUM = 0;			//Sum for Average
	float Enc_Val_raw;
	float Average;			//Encoder Value after Normalization from 0.25-4.08 to 0-1023
	int i = 0;					//value counter
	//bool state=1;
	while (i<=n)
		{
		  //uint8_t Enc_old; 
			Enc_Val_raw = 255;//ReadAnalogInput(ADC_IN1);//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);// READ SENSOR VALUE float value NOT digital value
			/**Normalization Exponential
			//___________________________________________________________________________________________________
			//data[n] = Enc_val;
			//			if(state) {Enc_old = Enc_Val_raw; state =0; }
			//			else {state=1;}
			//			//Enc_Val_raw = 90*Enc_Val_raw+(1-90)*Enc_old;//exponential filter doesnt function properly
			//--------------------------------------------------------------------------------------------------*/
			SUM = SUM + Enc_Val_raw;   					//Sum for Average 
			i = i+1;														//value counter
			
		}
	Average = SUM/n;
	//Normalize to 0-1023
	Average = Average - 63;		//Subtraction
	Average = Average / 972;   //Division
	Average = Average * 1023;   //GAIN
		
	Enc_Val_raw = ReadAnalogInput(ADC_IN1); //Raw data without filter 0-5000 mV
		
	return( Enc_Val_raw);//(long)Average); return value for PLC inverted 0-1023 must be output y = 1023-x
}

 int * Validaton(Enc_valid){
	 
	 int ArrEnc[]= {0,0,0,0,0,0,0,0,0,0};  // Dataset Array  for CANopen Communication default ={0}
	 
	 if(Enc_valid > 0 && Enc_valid < 1028) // Datavalidility check if range 0-1023+5 true {Enc_DataVal}
	 {
		 ArrEnc[0] = true; //Encoder value within range
	 }
	 else
	 {
		 ArrEnc[0] = false; //Encoder value outside range
	 }
	 
	 if(Enc_valid > 631 && Enc_valid < 1028) // TRACTION Pos active {TrBr_T} [5]=0 => OK; [5]=1 i=> FAIL;
	 {
		 ArrEnc[1] = true; //Confirm if S1 is activated and change to databit of array
		 if(ReadAnalogInput(ADC_IN3)>10){ArrEnc[5] = 0;}else{ArrEnc[5]=1;} //Check status of S1 {MICRO1_TrBr_Ko}
	 }
	 else
	 {
		 ArrEnc[1] = false;
		 if(ReadAnalogInput(ADC_IN3)>10){ArrEnc[5] = 1;}else{ArrEnc[5]=0;} //Confirm that S1 is deactivated outside range
	 }
	 
		if(Enc_valid > 541 && Enc_valid < 551) // IDLE Pos active {TrBr_Zero} [6]=0 => OK; [6]=1 i=> FAIL;
	 {
		 ArrEnc[2] = true;
		 if(HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin)){ArrEnc[6] = 0;}else{ArrEnc[6]=1;} //Check status of S2 {MICRO2_TrBr_Ko}
	 }
	 else
	 {
		 ArrEnc[2] = false;
		 if(HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin)){ArrEnc[6] = 1;}else{ArrEnc[6]=0;}//Confirm that S2 is deactivated outside range
	 }
	 /** BRAKE Switch S4 DIN6 (does not function with STG-820)
//	 if(Enc_valid > 109 && Enc_valid < 500) // BRAKE Pos active {TrBr_B}
//	 {
//		 ArrEnc[3] = true;
//		 if(HAL_GPIO_ReadPin(DIN6_Port,DIN6_Pin)){ArrEnc[8] = 0;}else{ArrEnc[8]=1;} //Check status of S4 {MICRO4_TrBr_Ko}
//	 }
//	 else
//	 {
//		 ArrEnc[3] = false;
//		 if(HAL_GPIO_ReadPin(DIN6_Port,DIN6_Pin)){ArrEnc[8] = 1;}else{ArrEnc[8]=0;}
//	 }
	 */
	 
	 	 
		if(Enc_valid > 0 && Enc_valid <= 10) // EMERGENCY Pos active {TrBr_EMG} [7]=1 => OK; [7]=0 i=> FAIL;
	 {
		 ArrEnc[4] = true;
		 if(HAL_GPIO_ReadPin(DIN5_Port,DIN5_Pin)){ArrEnc[7] = 1;}else{ArrEnc[7]=0;} //Check status of S3 {MICRO3_TrBr_Ko}
	 }
	 else
	 {
		 ArrEnc[4] = false;
		 if(HAL_GPIO_ReadPin(DIN5_Port,DIN5_Pin)){ArrEnc[7] = 0;}else{ArrEnc[7]=1;}//Confirm that S3 is activated outside range
	 }
	 if( ArrEnc[5] || ArrEnc[6] || ArrEnc[7] || ArrEnc[8] || FAILSTATE) //Check that no switches FAILED, will reset with restart
		{
			ArrEnc[9] = 0;//Microswitch Valid variable boolean bit in ERROR state
			FAILSTATE = true;
		} 
		else
		{
			ArrEnc[9]=1;//Microswitch Valid variable boolean bit in OK state
		}
		
	 return(ArrEnc);
 }


 
 uint8_t Dataset(bool b1,bool b2,bool b3,bool b4,bool b5,bool b6,bool b7,bool b8)
{ //Merge dataset boolean bits into int to transmit through CanOpen signal
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
		u8 += dataBits[7-i]*power; //Bin to dec calculation
		power *=2;
	}
	
	
	for (i=0;i<2;i++)
	{
		u8 += dataBits[1-i]*power;
		power *=2;
	}

		
		
/**	Bitshift example
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
	/** Disclaimer:
	Do not change the system clock above 16 MHz! Higher speed can lead to the destruction of the module!
	*/
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT; //Reset CanOpen
  
  MainInit();// System init	
	
	bool STAT2 = Initialize_outputs();// activate outputs and set to HIGH or 5V
	
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);// Main LED On
	SetAnalogOutput(5100);// Set analog output to 5V
	// Start DAC output
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2); // Start DAC channel for Digital analog read
	
	// =======================================================================
	// Set up baudrate to 250kb/s
	hcan.Instance->BTR &= 0xFFFFFC00;
	hcan.Instance->BTR |= CAN_250K;
	// =======================================================================
	//float Enc_Val_raw = ReadAnalogInput(ADC_IN1); //Read Sensor output with analog input
	
	/* increase variable each startup. Variable should be stored in EEPROM. */
	OD_powerOnCounter++;
 	bool STAT1 = Initialize_outputs();
  /* Infinite loop */
  while (1)
  {
		/** Sample code link
		//=======================================================================
		// CANOpen sample code
		// Untested port of https://github.com/CANopenNode/CANopenNode
		// =======================================================================*/
		
		while(reset != CO_RESET_APP) // while reset not active
		{
			/* CANopen communication reset - initialize CANopen objects */
			CO_ReturnError_t err; // Define Error Variable
			uint16_t timer1msPrevious;
      CanDisable();/* disable CAN and CAN interrupts */
			uint8_t NodeID1= 56; //Default set Node ID if Jumper open/FALSE
			bool NodeID_condition = 0; //Node ID change condition
			if (ReadAnalogInput(ADC_IN2))  //Condition for noe ID is HIGH / TRUE
			{
				uint8_t NodeID1 = 56; //CPU1-CAB2 = 58   CPU2-CAB2 = 59
				NodeID_condition = 1;
			}
			else
			{
				uint8_t NodeID1 = 58; //CPU1-CAB1 = 56   CPU2-CAB1 = 57
				NodeID_condition = 0;
			}
			/* initialize CANopen and check for errors */
			err = CO_init(0/* CAN module address */, NodeID1/* NodeID */, CAN_250K /* bit rate */);
			if(err != CO_ERROR_NO) //If not in error continue to infinite loop 
			{
					while(1)//infinite loop
					{
						int NodeID1 = 0; //Default set Node ID if Jumper open/FALSE
						bool NodeID = 0;
						if (ReadAnalogInput(ADC_IN2))  //Condition is TRUE if pin 13 of register c is HIGH / TRUE
						{
							int NodeID1 = 56; //CPU1-CAB2
							NodeID = 1;
						}
						else
						{
							int NodeID1 = 58; //CPU1-CAB1
							NodeID = 0;
						}
						
						
						STAT1 = Initialize_outputs();
						
						long Enc_Val_filtered = EN1_filter(100);//Readout sensor value 0.21-4.08V translate to 0-1023 and filter noise for n variables
						
						
						// LED flicker for error
						if ( u16Timer == 0 )
						{
							u16Timer = 100;
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


							// =======================================================================
							// Analog Output sample code
							// Increment Output voltage every one ms by 50 mV
							// If Output voltage overstepped 5100 mV it starts with 0
							// =======================================================================
							
//							u16Voltage = u16Voltage + 50;
//							if ( u16Voltage >= 5100 )
//								u16Voltage = 0;
						

						}
					}
					/* CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err); */
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

			while(reset == CO_RESET_NOT)
			{
				/* loop for normal program execution ******************************************/
				uint16_t timer1msCopy, timer1msDiff;

				timer1msCopy = CO_timer1ms;
				timer1msDiff = timer1msCopy - timer1msPrevious;
				timer1msPrevious = timer1msCopy;

				
				/* CANopen process */
	//			reset = CO_process(CO, timer1msDiff, NULL);

				/* Nonblocking application code may go here. */
				// LED handling:
				STAT1 = Initialize_outputs();
				
				// Send it by CAN
				hcan.pTxMsg->IDE = CAN_ID_STD;
				//========================================================================
				//Read analog inputs and call arithmitic function for n values of encoder
				//1. Moving average function + convert Float to LONG int 
				//2. Switch status readout
				//========================================================================
				if (NodeID_condition){hcan.pTxMsg->StdId = 0x00003A;   //Reciever adres: 0x003A (DMA-15)
					}
				else{hcan.pTxMsg->StdId = 0x000038;   //Reciever adres: 0x0038 (DMA-15)
					} 
				
				hcan.pTxMsg->DLC = 4 ;					
				uint16_t Enc_Val_filtered = EN1_filter(10);//Readout sensor value 0.21-4.08V translate to 0-1023 and filter noise for n variables
				
				//int * CAN_DATA[10] = {Validaton(Enc_Val_filtered)};//Function to validate the microswitches and encoder validility and convert them to an array
				int * CAN_DATA;
				CAN_DATA = Validaton(Enc_Val_filtered);//Function to validate the microswitches and encoder validility and convert them to an array

					//Debugging code:		
				int S1 = ReadAnalogInput(ADC_IN3);
				int S2 = HAL_GPIO_ReadPin(DIN4_Port,DIN4_Pin);
				int S3 = HAL_GPIO_ReadPin(DIN5_Port,DIN5_Pin);
				int S1234[3] = {S1,S2,S3};
				
				
				CanMSG.u32[0] = 0;
				CanMSG.u32[1] = 0;
				
				CanMSG.u8[0] = *((uint8_t*)&(Enc_Val_filtered)+1); //high byte (0x12)Enc_Val_filtered;
				CanMSG.u8[1] = *((uint8_t*)&(Enc_Val_filtered)+0); //low byte  (0x34)Enc_Val_filtered;
				bool Enc_Data_Val 	= CAN_DATA[0];//129   10000001
				bool TrBr_T 				= CAN_DATA[1];
				bool TrBr_Zero			= CAN_DATA[2];//163   10100011
				bool TrBr_B					= CAN_DATA[3];//197   11000101
				bool TrBr_EMG				= CAN_DATA[4];// 1    00000001
				bool MICRO1_TrBr_Ko	= CAN_DATA[5];
				bool MICRO2_TrBr_Ko	= CAN_DATA[6];
				bool MICRO3_TrBr_Ko	= CAN_DATA[7];//129   10000001
				bool MICRO4_TrBr_Ko	= CAN_DATA[8];
				bool TrBr_dataValid = CAN_DATA[9];	
				
				uint8_t dataset1 = Dataset(CAN_DATA[0],CAN_DATA[1],CAN_DATA[2],CAN_DATA[3],CAN_DATA[4],CAN_DATA[5],CAN_DATA[6],CAN_DATA[7]);
				uint8_t dataset2 = Dataset(0,0,0,0,0,0,CAN_DATA[8],CAN_DATA[9]);
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
				HAL_CAN_Transmit(&hcan, 25);
				
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
