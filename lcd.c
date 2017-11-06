/*
 * lcd.c
 *
 *  Created on: Nov 2, 2017
 *      Author: L01073411
 */
#include "ee.h"
#include "ee_irq.h"
#define FCY 40000000UL
#include <libpic30.h>

#include "lcd.h"

/* Writes an initial message in the LCD display first row */
void put_LCD_initial_message()
{
	EE_lcd_goto( 0, 0 );

	EE_lcd_putc('S');
	EE_lcd_putc('E');
	EE_lcd_putc('R');
	EE_lcd_putc('I');
	EE_lcd_putc('A');
	EE_lcd_putc('L');
	EE_lcd_putc(' ');
	EE_lcd_putc('N');
	EE_lcd_putc('D');
	EE_lcd_putc('V');
	EE_lcd_putc('I');
	EE_lcd_putc(' ');
	EE_lcd_putc(' ');
	EE_lcd_putc(' ');
	EE_lcd_putc(' ');
	EE_lcd_putc(' ');

}

/* Writes an message in the LCD display second row terminated with a null character*/
void put_LCD_string_message(char message[])
{
	int i=0;
	EE_lcd_line2();
	while(message[i]>0){
		EE_lcd_putc(message[i++]);
	}

}

/* Writes an NVDI message in the LCD display second row */
void put_LCD_nvdi_data(char message[],unsigned char data_offset,unsigned char data_size)
{
	int i=0;
	EE_lcd_line2();
	for(i=0; i<data_size; i++){
		EE_lcd_putc(message[data_offset+i]);
	}
}



