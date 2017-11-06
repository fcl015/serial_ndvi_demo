/*
 * ndvi.c
 *
 *  Created on: Nov 6, 2017
 *      Author: L01073411
 */

#include <string.h>
#include <stdlib.h>

#include "ndvi.h"

float Obtain_NDVI_Value(char rawdata[])
{
	float value1;
	float value2;
	char data1[5];
	char data2[5];

	strncpy(data1,&rawdata[2],4);
	data1[4]=0;
	strncpy(data2,&rawdata[7],4);
	data2[4]=0;

	value1=atof(data1);
	value2=atof(data2);

	return(value1+value2);
}



