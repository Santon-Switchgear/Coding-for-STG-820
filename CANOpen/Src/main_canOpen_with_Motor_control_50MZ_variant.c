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
//latest
/* Private variables ---------------------------------------------------------*/
CanTxMsgTypeDef CAN_TX_Msg; //Variablen, die zur Struktur "hcan" des HAL-Treibers              
CanRxMsgTypeDef CAN_RX_Msg; //geh�ren
__IO uint16_t u16Timer = 0; //Z�hlvariable Softwaretimer 1
__IO uint16_t u16Timer1 = 0;//Z�hlvariable Softwaretimer 2
uint16_t u16Tx = 0; //Variable f�r das Erzeugen einer zuf�lligen CAN-Sendeverz�gerung
uint8_t Freigabe = 0; //Freigabe-Variable: 0=Motor Stromlos; 1=Bremsen/Drehen
uint8_t Strom = 0x11; //Motorstrom-Variable: 0x11=17= 1,7A
uint8_t SchrittZahlL = 0x18; //Schrittzahl Low-Byte
uint8_t SchrittZahlH = 0xFC; //Schrittzahl High-Byte 
//Low- und High-Byte ergeben gemeinsam: 0xFC18 = -1000
uint8_t MinimalFrequenzL = 0x32; //Minimalfrequenz (Drehzahl) Low-Byte
uint8_t MinimalFrequenzH = 0x00; //Minimalfrequenz (Drehzahl) High-Byte
//Low- und High-Byte ergeben gemeinsam: 0x0032 = 50
uint8_t MaximalFrequenzL = 0xC8; //Maximalfrequenz (Drehzahl) Low-Byte
uint8_t MaximalFrequenzH = 0x00; //Maximalfrequenz (Drehzahl) High-Byte
//Low- und High-Byte ergeben gemeinsam: 0x00C8 = 200

/* Private function prototypes -----------------------------------------------*/

void HAL_SYSTICK_Callback(void) //Funktion des Treibers, die jede Millisekunde aufgerufen wird
{
	/*
	Folgender Programmteil z�hlt zwei Variablen pro Aufruf um 1 herunter. 
	Auf diese Weise kann, unabh�ngig von der Hardware, eine gro�e Anzahl 
	von vergleichsweise unpr�zisen Software-Timern erzeugt werden.
	Sie k�nnen genutzt werden, um Funktionen in unterschiedlichen Zeitabst�nden aufzurufen. 
	*/
  if ( u16Timer > 0 )
		u16Timer--;
  if ( u16Timer1 > 0 )
		u16Timer1--;
}

#define WAIT1 u16Timer1=10;while(u16Timer1);//Aufruf von "WAIT1" erzeugt eine 
                                             //Verz�gerung von 10ms, indem u16Timer1
                                             //auf 10 gesetzt wird
void ShowTemplate1 ( void ) //Funktion, die Vorlage "Arrows" (Pfeilkreuz) 
	                          //�ber den CAN-Bus aufruft
{
	/*
	Folgender Programmteil erzeugt eine zuf�llige Zeitverz�gerung zwischen
	Aufruf der Funktion und Senden der CAN-Nachricht. Hierdurch wird das Senden von
	Nachrichten zeitlich vom Programmablauf so entkoppelt, dass zwei Nachrichten nicht 
	regelm��ig zum gleichen Zeitpunkt gesendet werden k�nnen; dieser Zustand w�rde die
	Buskommunikation massiv st�ren und der Fehler w�re nicht einfach nachvollziehbar. 
	*/
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;

	/*
	Folgender Programmteil definiert Format und Wert der Empf�ngeradresse, 
	sowie die Nachrichtenl�nge.
	*/
	hcan.pTxMsg->IDE = CAN_ID_STD;//Standard-ID
	hcan.pTxMsg->StdId = 0x7FD;   //Empf�ngeradresse: 0x7FD (DMA-15)
	hcan.pTxMsg->DLC = 8;         //Nachrichtenl�nge: 8 Byte
	
	
	hcan.pTxMsg->Data[0] = 0x09;  //Vorlage "1V" auf DMA-15 aufrufen
	hcan.pTxMsg->Data[1] = 0x00;  //Nicht genutzt
	hcan.pTxMsg->Data[2] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[3] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[4] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[5] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[6] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[7] = 0xDF;  //Alle Schaltfl�chen der Vorlage darstellen
	HAL_CAN_Transmit(&hcan, 10); WAIT1 //10ms Zeitverz�gerung f�r sicheres Senden
}

void StepperMessage ( void ) //Funktion, die den Schrittmotortreiber 
														 //eine bestimmte Anzahl von Schritten mit
														 //einer bestimmten Geschwindigkeit in eine
														 //Bestimmte Richtung ausf�hren l�sst
{
	/*
	Folgender Programmteil erzeugt eine zuf�llige Zeitverz�gerung zwischen
	Aufruf der Funktion und Senden der CAN-Nachricht. Hierdurch wird das Senden von
	Nachrichten zeitlich vom Programmablauf so entkoppelt, dass zwei Nachrichten nicht 
	regelm��ig zum gleichen Zeitpunkt gesendet werden k�nnen; dieser Zustand w�rde die
	Buskommunikation massiv st�ren und der Fehler w�re nicht einfach nachvollziehbar. 
	*/
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;
	
	/*
	Folgender Programmteil definiert Format und Wert der Empf�ngeradresse, 
	sowie die Nachrichtenl�nge.
	*/
	hcan.pTxMsg->IDE = CAN_ID_STD; //Standard-ID
	hcan.pTxMsg->StdId = 0x201;    //CAN-ID des Motorcontrollers f�r Anfahren einer bestimmten Position
	hcan.pTxMsg->DLC = 8;          //Nachrichtenl�nge in Byte
	
	/*
	Folgender Programmteil schreibt Byte 0-7 in das Array "Data".
	Es wird dann in den "CAN-Ausgangspuffer" (pTxMsg) geladen und  
	durch den Befehl "HAL_CAN_Transmit" gesendet. Byte 7 wird hierbei zuerst und Byte 0
	zuletzt �bertragen. 
	*/
	hcan.pTxMsg->Data[0] = Freigabe; //Freigabevariable �bergeben 
	hcan.pTxMsg->Data[1] = Strom;    //Stromvariable �bergeben
	hcan.pTxMsg->Data[2] = SchrittZahlL; //Schrittzahl Low-Byte �bergeben
	hcan.pTxMsg->Data[3] = SchrittZahlH; //Schrittzahl High-Byte �bergeben
	hcan.pTxMsg->Data[4] = MinimalFrequenzL; //MinimalFrequenz Low-Byte �bergeben
	hcan.pTxMsg->Data[5] = MinimalFrequenzH; //MinimalFrequenz High-Byte �bergeben
	hcan.pTxMsg->Data[6] = MaximalFrequenzL; //MaximalFrequenz Low-Byte �bergeben
	hcan.pTxMsg->Data[7] = MaximalFrequenzH; //MaximalFrequenz Low-Byte �bergeben
	HAL_CAN_Transmit(&hcan, 10); WAIT1 //10ms Zeitverz�gerung f�r sicheres Senden
}

/*Folgende Funktion verarbeitet die R�ckmeldung des Displays beim Ber�hren einer Schaltfl�che*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if ( hcan->pRxMsg->IDE == CAN_ID_STD)//Wenn das Adressformat der Nachricht
		                                   //dem Standardformat entspricht, wird 
	                                     //diese Schleife betreten
	{
		switch ( hcan->pRxMsg->StdId )     //Diese Funktion unterscheidet, 
			                                 //von welcher Adresse die Nachricht stammt
		{
			case 0x7FC: //In diesem Fall werden Nachrichten vom Display verarbeitet
			
				if(hcan->pRxMsg->Data[0] == 0x09) //Hiermit wird erkannt, auf welcher 
					                                //Vorlage die Schaltfl�che bet�tigt wurde
					                                
				{
					if(hcan->pRxMsg->Data[2] == 0x01) //Wenn "Pfeil hoch" bet�tigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verz�gerung von 100ms
						StepperMessage();//Sperrbefehl senden
						MaximalFrequenzL = 0xE8; //Werte f�r neue Drehzahl Vorgeben:
	          MaximalFrequenzH = 0x03; //0x03E8 = 1000Hz
						
					}
			
					if(hcan->pRxMsg->Data[2] == 0x02) //Wenn "Pfeil rechts" bet�tigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verz�gerung von 100ms
						StepperMessage();//Sperrbefehl senden
						SchrittZahlL = 0xE8; //Werte f�r neue Drehrichtung Vorgeben:
            SchrittZahlH = 0x03; //0x03E8 = 1000 Schritte im Uhrzeigersinn
					}
				
					if(hcan->pRxMsg->Data[2] == 0x04) //Wenn "Pfeil runter" bet�tigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verz�gerung von 100ms
						StepperMessage();//Sperrbefehl senden
						MaximalFrequenzL = 0xC8; //Werte f�r neue Drehzahl Vorgeben:
	          MaximalFrequenzH = 0x00; //0x00C8 = 200Hz
					}
				
					if(hcan->pRxMsg->Data[2] == 0x08) //Wenn "Pfeil links" bet�tigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verz�gerung von 100ms
						StepperMessage();//Sperrbefehl senden
						SchrittZahlL = 0x18; //Werte f�r neue Drehrichtung Vorgeben:
            SchrittZahlH = 0xFC; //0xFC18 = -1000 Schritte (gegen den Uhrzeigersinn)
					}
				
					if(hcan->pRxMsg->Data[2] == 0x40) //Wenn "ESC" bet�tigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verz�gerung von 100ms
						StepperMessage();//Sperrbefehl senden
					}
					
					if(hcan->pRxMsg->Data[2] == 0x80) //Wenn "on/off" bet�tigt...
					{
						Freigabe ^= (1 << 0); //Antrieb sperren oder freigeben
						HAL_Delay(100);       //Verz�gerung von 100ms
						StepperMessage();     //Sperr- bzw. Freigabebefehl senden
					}
				}
					break;
		}
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0); //Interrupt-Flags zur�cksetzen und Interrupt aktivieren
}



int main(void) //Hauptprogrammschleife
{
  // System init	
  MainInit(); //Diese Funktion befindet sich in der Datei "main_hal.c".
							//Sie initialisiert die ben�tigten Peripheriekomponenten, 
	            //wie IO, CAN-Controller, Timer, etc. 
	ShowTemplate1(); //Bei Programmstart auf Vorlage 1 springen
	
	
	// =======================================================================
	// Baudrate auf 250k einstellen
	hcan.Instance->BTR &= 0xFFFFFC00;
	hcan.Instance->BTR |= CAN_250K;
	// =======================================================================
	
	
  
  while (1)//Endlosschleife
  {
		//Das Programm l�uft vollst�ndig in den definierten Funktionen ab, 
		//hier wird daher kein Code ausgef�hrt
	}
				
				
/*
	Wenn PRODUCTION_VERSION 1 ist, ist der Watchdog-Timer aktiv und l�st einen Reset aus,
	wenn er nicht innerhalb eines bestimmten Zeitraumes zur�ckgesetzt wird. 
	In diesem Fall warnt der folgende Programmeil im Fenster "Build Output", dass Debuggen unter dieser Bedingung 
	nicht m�glich ist. Das hat den Grund, dass beim Debuggen die Programmausf�hrung 
	durch das Programmierger�t zeitweise angehalten wird. Ein regelm��iges 
	R�cksetzen des Watchdogs ist daher nicht m�glich. 
	
	Wenn PRODUCTION_VERSION 0 ist, ist der Watchdog-Timer deaktiviert. 
	Weil der Watchdog die zuverl�ssige Programmausf�hrung unterst�tzt, sollte er in
	kommerziellen Anwendungen meist verwendet werden. Es wird daher darauf hingewiesen, 
	dass der Watchdog zur Fehlersuche deaktiviert wurde. 
*/
		#if ( PRODUCTION_VERSION == 1 )
		  HAL_IWDG_Refresh(&hiwdg);
			#warning Production version, Debugging not possible! <<<<<<<<<<<<<<<<<<<<<
		#else
			#warning Debug version without watch dog! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		#endif
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/