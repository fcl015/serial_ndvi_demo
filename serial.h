/*
 * serial.h
 *
 *  Created on: Nov 2, 2017
 *      Author: L01073411
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void Serial1_Init(void);
void Serial2_Init(void);
int Serial_Send(unsigned char data, unsigned char port);
int Serial_Send_Frame(char *ch, unsigned char len, unsigned char port);
int Serial_Receive(char* data, unsigned char port);
int Serial_Receive_Frame(char *ch, unsigned char len, unsigned char port);
TASK(Task_TimeOut);


#endif /* SERIAL_H_ */
