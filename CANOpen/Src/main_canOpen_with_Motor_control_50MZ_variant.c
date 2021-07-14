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
CanRxMsgTypeDef CAN_RX_Msg; //gehören
__IO uint16_t u16Timer = 0; //Zählvariable Softwaretimer 1
__IO uint16_t u16Timer1 = 0;//Zählvariable Softwaretimer 2
uint16_t u16Tx = 0; //Variable für das Erzeugen einer zufälligen CAN-Sendeverzögerung
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
	Folgender Programmteil zählt zwei Variablen pro Aufruf um 1 herunter. 
	Auf diese Weise kann, unabhängig von der Hardware, eine große Anzahl 
	von vergleichsweise unpräzisen Software-Timern erzeugt werden.
	Sie können genutzt werden, um Funktionen in unterschiedlichen Zeitabständen aufzurufen. 
	*/
  if ( u16Timer > 0 )
		u16Timer--;
  if ( u16Timer1 > 0 )
		u16Timer1--;
}

#define WAIT1 u16Timer1=10;while(u16Timer1);//Aufruf von "WAIT1" erzeugt eine 
                                             //Verzögerung von 10ms, indem u16Timer1
                                             //auf 10 gesetzt wird
void ShowTemplate1 ( void ) //Funktion, die Vorlage "Arrows" (Pfeilkreuz) 
	                          //über den CAN-Bus aufruft
{
	/*
	Folgender Programmteil erzeugt eine zufällige Zeitverzögerung zwischen
	Aufruf der Funktion und Senden der CAN-Nachricht. Hierdurch wird das Senden von
	Nachrichten zeitlich vom Programmablauf so entkoppelt, dass zwei Nachrichten nicht 
	regelmäßig zum gleichen Zeitpunkt gesendet werden können; dieser Zustand würde die
	Buskommunikation massiv stören und der Fehler wäre nicht einfach nachvollziehbar. 
	*/
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;

	/*
	Folgender Programmteil definiert Format und Wert der Empfängeradresse, 
	sowie die Nachrichtenlänge.
	*/
	hcan.pTxMsg->IDE = CAN_ID_STD;//Standard-ID
	hcan.pTxMsg->StdId = 0x7FD;   //Empfängeradresse: 0x7FD (DMA-15)
	hcan.pTxMsg->DLC = 8;         //Nachrichtenlänge: 8 Byte
	
	
	hcan.pTxMsg->Data[0] = 0x09;  //Vorlage "1V" auf DMA-15 aufrufen
	hcan.pTxMsg->Data[1] = 0x00;  //Nicht genutzt
	hcan.pTxMsg->Data[2] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[3] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[4] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[5] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[6] = 0x00;	//Nicht genutzt
	hcan.pTxMsg->Data[7] = 0xDF;  //Alle Schaltflächen der Vorlage darstellen
	HAL_CAN_Transmit(&hcan, 10); WAIT1 //10ms Zeitverzögerung für sicheres Senden
}

void StepperMessage ( void ) //Funktion, die den Schrittmotortreiber 
														 //eine bestimmte Anzahl von Schritten mit
														 //einer bestimmten Geschwindigkeit in eine
														 //Bestimmte Richtung ausführen lässt
{
	/*
	Folgender Programmteil erzeugt eine zufällige Zeitverzögerung zwischen
	Aufruf der Funktion und Senden der CAN-Nachricht. Hierdurch wird das Senden von
	Nachrichten zeitlich vom Programmablauf so entkoppelt, dass zwei Nachrichten nicht 
	regelmäßig zum gleichen Zeitpunkt gesendet werden können; dieser Zustand würde die
	Buskommunikation massiv stören und der Fehler wäre nicht einfach nachvollziehbar. 
	*/
	u16Tx = u16Tx + 10 + (rand() & 0xFFF);
	if (u16Tx > 30000)
		u16Tx = 1;
	
	/*
	Folgender Programmteil definiert Format und Wert der Empfängeradresse, 
	sowie die Nachrichtenlänge.
	*/
	hcan.pTxMsg->IDE = CAN_ID_STD; //Standard-ID
	hcan.pTxMsg->StdId = 0x201;    //CAN-ID des Motorcontrollers für Anfahren einer bestimmten Position
	hcan.pTxMsg->DLC = 8;          //Nachrichtenlänge in Byte
	
	/*
	Folgender Programmteil schreibt Byte 0-7 in das Array "Data".
	Es wird dann in den "CAN-Ausgangspuffer" (pTxMsg) geladen und  
	durch den Befehl "HAL_CAN_Transmit" gesendet. Byte 7 wird hierbei zuerst und Byte 0
	zuletzt übertragen. 
	*/
	hcan.pTxMsg->Data[0] = Freigabe; //Freigabevariable übergeben 
	hcan.pTxMsg->Data[1] = Strom;    //Stromvariable übergeben
	hcan.pTxMsg->Data[2] = SchrittZahlL; //Schrittzahl Low-Byte übergeben
	hcan.pTxMsg->Data[3] = SchrittZahlH; //Schrittzahl High-Byte übergeben
	hcan.pTxMsg->Data[4] = MinimalFrequenzL; //MinimalFrequenz Low-Byte übergeben
	hcan.pTxMsg->Data[5] = MinimalFrequenzH; //MinimalFrequenz High-Byte übergeben
	hcan.pTxMsg->Data[6] = MaximalFrequenzL; //MaximalFrequenz Low-Byte übergeben
	hcan.pTxMsg->Data[7] = MaximalFrequenzH; //MaximalFrequenz Low-Byte übergeben
	HAL_CAN_Transmit(&hcan, 10); WAIT1 //10ms Zeitverzögerung für sicheres Senden
}

/*Folgende Funktion verarbeitet die Rückmeldung des Displays beim Berühren einer Schaltfläche*/
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
					                                //Vorlage die Schaltfläche betätigt wurde
					                                
				{
					if(hcan->pRxMsg->Data[2] == 0x01) //Wenn "Pfeil hoch" betätigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verzögerung von 100ms
						StepperMessage();//Sperrbefehl senden
						MaximalFrequenzL = 0xE8; //Werte für neue Drehzahl Vorgeben:
	          MaximalFrequenzH = 0x03; //0x03E8 = 1000Hz
						
					}
			
					if(hcan->pRxMsg->Data[2] == 0x02) //Wenn "Pfeil rechts" betätigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verzögerung von 100ms
						StepperMessage();//Sperrbefehl senden
						SchrittZahlL = 0xE8; //Werte für neue Drehrichtung Vorgeben:
            SchrittZahlH = 0x03; //0x03E8 = 1000 Schritte im Uhrzeigersinn
					}
				
					if(hcan->pRxMsg->Data[2] == 0x04) //Wenn "Pfeil runter" betätigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verzögerung von 100ms
						StepperMessage();//Sperrbefehl senden
						MaximalFrequenzL = 0xC8; //Werte für neue Drehzahl Vorgeben:
	          MaximalFrequenzH = 0x00; //0x00C8 = 200Hz
					}
				
					if(hcan->pRxMsg->Data[2] == 0x08) //Wenn "Pfeil links" betätigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verzögerung von 100ms
						StepperMessage();//Sperrbefehl senden
						SchrittZahlL = 0x18; //Werte für neue Drehrichtung Vorgeben:
            SchrittZahlH = 0xFC; //0xFC18 = -1000 Schritte (gegen den Uhrzeigersinn)
					}
				
					if(hcan->pRxMsg->Data[2] == 0x40) //Wenn "ESC" betätigt...
					{
						Freigabe = 0;    //Antrieb sperren
						HAL_Delay(100);  //Verzögerung von 100ms
						StepperMessage();//Sperrbefehl senden
					}
					
					if(hcan->pRxMsg->Data[2] == 0x80) //Wenn "on/off" betätigt...
					{
						Freigabe ^= (1 << 0); //Antrieb sperren oder freigeben
						HAL_Delay(100);       //Verzögerung von 100ms
						StepperMessage();     //Sperr- bzw. Freigabebefehl senden
					}
				}
					break;
		}
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0); //Interrupt-Flags zurücksetzen und Interrupt aktivieren
}



int main(void) //Hauptprogrammschleife
{
  // System init	
  MainInit(); //Diese Funktion befindet sich in der Datei "main_hal.c".
							//Sie initialisiert die benötigten Peripheriekomponenten, 
	            //wie IO, CAN-Controller, Timer, etc. 
	ShowTemplate1(); //Bei Programmstart auf Vorlage 1 springen
	
	
	// =======================================================================
	// Baudrate auf 250k einstellen
	hcan.Instance->BTR &= 0xFFFFFC00;
	hcan.Instance->BTR |= CAN_250K;
	// =======================================================================
	
	
  
  while (1)//Endlosschleife
  {
		//Das Programm läuft vollständig in den definierten Funktionen ab, 
		//hier wird daher kein Code ausgeführt
	}
				
				
/*
	Wenn PRODUCTION_VERSION 1 ist, ist der Watchdog-Timer aktiv und löst einen Reset aus,
	wenn er nicht innerhalb eines bestimmten Zeitraumes zurückgesetzt wird. 
	In diesem Fall warnt der folgende Programmeil im Fenster "Build Output", dass Debuggen unter dieser Bedingung 
	nicht möglich ist. Das hat den Grund, dass beim Debuggen die Programmausführung 
	durch das Programmiergerät zeitweise angehalten wird. Ein regelmäßiges 
	Rücksetzen des Watchdogs ist daher nicht möglich. 
	
	Wenn PRODUCTION_VERSION 0 ist, ist der Watchdog-Timer deaktiviert. 
	Weil der Watchdog die zuverlässige Programmausführung unterstützt, sollte er in
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