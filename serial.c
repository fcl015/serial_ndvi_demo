/*
 * serial.c
 *
 *  Created on: Nov 2, 2017
 *      Author: L01073411
 */

#include "ee.h"
#include "ee_irq.h"
#define FCY 40000000UL
#include <libpic30.h>

#include "serial.h"

volatile unsigned char comm_timeout_flag;
const unsigned int  comm_wait_time = 10000;

/******************************************************************************************
 * Función:	Serial1_Init()						     										  *
 * Descripción:	Configure Serial Port 1			 		          								  *
 ******************************************************************************************/
void Serial1_Init(void)
{
	/* Stop UART port */
	U1MODEbits.UARTEN = 0;

	/* Disable Interrupts */
	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;

	/* Clear Interrupt flag bits */
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;

	/* Set IO pins */
	//TRISDbits.TRISD14 = 0;  // CTS Output
	//TRISDbits.TRISD15 = 0;  // RTS Output
	TRISFbits.TRISF3 = 0;   // TX Output
	TRISFbits.TRISF2 = 1;   // RX Input

	/* baud rate */
	U1MODEbits.BRGH = 0;
	//U1BRG  = 259; // 9600 baud rate
	U1BRG  = 2082; // 1200 baud rate

	/* Operation settings and start port */
	U1MODE = 0;
	U1MODEbits.UEN = 0; //2
	U1MODEbits.UARTEN = 1;

	/* TX & RX interrupt modes */
	U1STA = 0;
	U1STAbits.UTXEN=1;
}


/******************************************************************************************
 * Función:	Serial2_Init()						     										  *
 * Descripción:	Configure Serial Port 2			 		          								  *
 ******************************************************************************************/
void Serial2_Init(void)
{
	/* Stop UART port */
	U2MODEbits.UARTEN = 0;

	/* Disable Interrupts */
	IEC1bits.U2RXIE = 0;
	IEC1bits.U2TXIE = 0;

	/* Clear Interrupt flag bits */
	IFS1bits.U2RXIF = 0;
	IFS1bits.U2TXIF = 0;

	/* Set IO pins */
	//TRISFbits.TRISF12 = 0;  // CTS Output
	//TRISFbits.TRISF13 = 0;  // RTS Output
	TRISFbits.TRISF5 = 0;   // TX Output
	TRISFbits.TRISF4 = 1;   // RX Input

	/* baud rate */
    U2MODEbits.BRGH = 0;
	//U2BRG  = 259; // 9600 baud rate
	U2BRG  = 2082; // 1200 baud rate

	/* Operation settings and start port */
	U2MODE = 0;
	U2MODEbits.UEN = 0; //2
	U2MODEbits.UARTEN = 1;

	/* TX & RX interrupt modes */
	U2STA = 0;
	U2STAbits.UTXEN=1;
}


/******************************************************************************************
 * Send one byte via serial port					     										  *
 ******************************************************************************************/
int Serial_Send(unsigned char data, unsigned char port)
{
	switch(port){
		case 1:		while (U1STAbits.UTXBF);
					U1TXREG = data;
					while(!U1STAbits.TRMT);
					return 0;
					break;
		case 2:		while (U2STAbits.UTXBF);
					U2TXREG = data;
					while(!U2STAbits.TRMT);
					return 0;
					break;
		default:	return -1;
					break;
	}
}

/******************************************************************************************
 * Send a data frame via serial port					     										  *
 ******************************************************************************************/
int Serial_Send_Frame(char *ch, unsigned char len, unsigned char port)
{
    char i;

    for (i = 0; i < len; i++) {
    	Serial_Send(*(ch++),port);
    }
    return i;
}


/******************************************************************************************
 * Receive one byte via serial port
 * RETURNS
 * 		 0 	Data is available and it is read
 * 		-1	No data available
 * 		-2  Data overflow
 * 		-3  Incorrect port    										  *
 ******************************************************************************************/
int Serial_Receive(char* data, unsigned char port)
{
	switch(port){
		// Receive character from port 1
		case 1:		// 1 = Receive buffer has overflowed,    0 = Receive buffer has not overflowed
					if (U1STAbits.OERR) {
							U1STAbits.OERR = 0;
							LATAbits.LATA0=1;
							return -2;
					}
					//1 = Receive buffer has data,  at least one more character can be read,
					//0 = Receive buffer is empty
					if (U1STAbits.URXDA) {
							*data = U1RXREG & 0x00FF;
							LATAbits.LATA0=0;
							return 0;
					}
					else {
							*data = 0;
							return -1;
					}
					break;
		// Receive character from port 2
		case 2:		// 1 = Receive buffer has overflowed,    0 = Receive buffer has not overflowed
					if (U2STAbits.OERR) {
							U2STAbits.OERR = 0;
							LATAbits.LATA0=1;
							return -2;
					}
					//1 = Receive buffer has data,  at least one more character can be read,
					//0 = Receive buffer is empty
					if (U2STAbits.URXDA) {
							*data = U2RXREG & 0x00FF;
							LATAbits.LATA0=0;
							return 0;
					}
					else {
							*data = 0;
							return -1;
					}
					break;
		// No serial port defined
		default:	*data = 0;
					return -1;
					break;
	}

}

/******************************************************************************************
 * Activate timeout flag if serial data is not received			     										  *
 ******************************************************************************************/
TASK(Task_TimeOut){
    comm_timeout_flag=1;
}

/******************************************************************************************
 * Receive a data frame via serial port
 * RETURNS
 * 		0		Communication error
 * 		1		Successfully message received				     										  *
 ******************************************************************************************/
int Serial_Receive_Frame(char *ch, unsigned char len, unsigned char port)
{
    char i;

    for (i = 0; i < len; i++) {
    	// Activate timeout task
    	comm_timeout_flag=0;
    	SetRelAlarm(AlarmTimeOut, comm_wait_time, 0);
    	switch(port){
    		// Receive frame from port 1
    		case 1:		while( !IFS0bits.U1RXIF && !comm_timeout_flag );
    					while( Serial_Receive(ch++,port) && !comm_timeout_flag );
    			        if(!comm_timeout_flag)
    			        	IFS0bits.U1RXIF = 0;
    			        else{
    			        	comm_timeout_flag=0;
    			            return 0;
    			        }
    					break;
    		// Receive frame from port 2
    		case 2: 	while( !IFS1bits.U2RXIF && !comm_timeout_flag );
    					while( Serial_Receive(ch++,port) && !comm_timeout_flag );
    					if(!comm_timeout_flag)
				        	IFS1bits.U2RXIF = 0;
				        else{
				        	comm_timeout_flag=0;
				            return 0;
				        }
    					break;
    		// Serial port not defined
    		default:	return 0;
    					break;
    	}
    	// Cancel timeout task
    	CancelAlarm(AlarmTimeOut);
    }
    return 1;
}



