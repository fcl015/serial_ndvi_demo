/*
 * code.c
 *
 *  Created on: 26/08/2014
 *      Author: L01073411
 */


#include "ee.h"
#include "ee_irq.h"
#define FCY 40000000UL
#include <libpic30.h>
#include <stdio.h>

#include "lcd.h"
#include "serial.h"
#include "ndvi.h"

// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystanl
_FOSC(OSCIOFNC_ON & POSCMD_XT);
// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);
// Disable Code Protection
_FGS(GCP_OFF);


/* Program the Timer1 peripheral to raise interrupts */
void T1_program(void)
{
	T1CON = 0;		/* Stops the Timer1 and reset control reg	*/
	TMR1  = 0;		/* Clear contents of the timer register	*/
	PR1   = 0x9c40;		/* PR1=40000 Load the Period register with the value of 1ms	*/
	//PR1   = 0x4e20;		/* PR1=20000 Load the Period register with the value of 0.5ms	*/
	//PR1   = 0x0fa0;		/* PR1=4000 Load the Period register with the value of 0.1ms	*/
	IPC0bits.T1IP = 5;	/* Set Timer1 priority to 1		*/
	IFS0bits.T1IF = 0;	/* Clear the Timer1 interrupt status flag	*/
	IEC0bits.T1IE = 1;	/* Enable Timer1 interrupts		*/
	T1CONbits.TON = 1;	/* Start Timer1 with prescaler settings at 1:1
						* and clock source set to the internal
						* instruction cycle			*/
}

/* This is an ISR Type 2 which is attached to the Timer 1 peripheral IRQ pin
 * The ISR simply calls CounterTick to implement the timing reference
 */
ISR2(_T1Interrupt)
{
	/* clear the interrupt source */
	IFS0bits.T1IF = 0;
	/* count the interrupts, waking up expired alarms */
	CounterTick(myCounter);
}


/******************************************************************************************
 * TASKS		     										  *
 ******************************************************************************************/

// TASK 1:  Blink leds every 1 second
TASK(Task1)
{
	put_LCD_initial_message();
	LATAbits.LATA4^=1;
}

// TASK 2: Not Implemented
TASK(Task2)
{
}

// TASK 3: Read NVDI Sensor from Serial Port
static char bufferIn[80];
unsigned char nvdi_total_message_size=17;
unsigned char nvdi_serial_port=1;
unsigned char nvdi_data_offset=2;
unsigned char nvdi_data_size=9;
static float ndvi_value;
static char ndvi_string[10];

// TASK 3: Read NVDI Sensor from Serial Port
TASK(Task3)
{
	// Activate signal (LED)
	LATAbits.LATA2=1;
	LATAbits.LATA15=1;
	__delay_ms(30);
	EE_lcd_clear();

	// Wait until message received
	Serial_Receive_Frame(bufferIn,nvdi_total_message_size,nvdi_serial_port);

	// Process data
	ndvi_value=Obtain_NDVI_Value(bufferIn);

	// Write received data on display
	sprintf(ndvi_string,"%5.2f",ndvi_value);
	put_LCD_nvdi_data(ndvi_string,0,5);
	//put_LCD_nvdi_data(bufferIn,nvdi_data_offset,nvdi_data_size);

	// Deactivate signal (LED)
	LATAbits.LATA15=0;
	LATAbits.LATA2=0;
}


//*****************************************************************************************
// Main routine
//*****************************************************************************************

int main(void)
{
	/* Clock setup for 40MIPS */
	/* PLL Configuration */
	PLLFBD=38; 				// M=40
	CLKDIVbits.PLLPOST=0; 	// N1=2
	CLKDIVbits.PLLPRE=0; 	// N2=2
	OSCTUN=0; 				// FRC clock use
	RCONbits.SWDTEN=0; 		//watchdog timer disable
	while(OSCCONbits.LOCK!=1); //wait for PLL LOCK

	/* Program Timer 1 to raise interrupts */
	T1_program();

	/* Init leds */
	TRISAbits.TRISA0=0;
	TRISAbits.TRISA1=0;
	TRISAbits.TRISA2=0;
	TRISAbits.TRISA3=0;
	TRISAbits.TRISA4=0;
	TRISAbits.TRISA15=0;
	LATAbits.LATA15=0;

	/* Init LCD */
	EE_lcd_init();
	EE_lcd_clear();

	/* Modules init */
	Serial1_Init();
	Serial2_Init();

	/* Program cyclic alarms which will fire after an initial offset, and after that periodically */
	SetRelAlarm(Alarm1,  1000,  1000);
	SetRelAlarm(Alarm2,  1000,  0);
	SetRelAlarm(Alarm3,  10500, 30000);

	 /* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}
