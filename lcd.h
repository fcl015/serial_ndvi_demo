/*
 * lcd.h
 *
 *  Created on: Nov 2, 2017
 *      Author: L01073411
 */

#ifndef LCD_H_
#define LCD_H_

void put_LCD_initial_message();
void put_LCD_string_message(char message[]);
void put_LCD_nvdi_data(char message[],unsigned char data_offset,unsigned char data_size);


#endif /* LCD_H_ */
