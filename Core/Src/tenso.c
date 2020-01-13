/*
 * tenso.c
 *
 *  Created on: 23.09.2019
 *      Author: Konto_U¿ytkowe
 */

#include "tenso.h"

HX711 data = {0, 0, 0, DATA_GAIN_FACTOR};


int HX711_Average_Value(uint8_t times)
{
    int sum = 0;
    int i = 0;
    for (i = 0; i < times; i++)
    {
        sum += HX711_Value();
    }

    return sum / times;
}

int HX711_Value()
{
    int buffer;
    buffer = 0;

    while (HAL_GPIO_ReadPin(gpioData_GPIO_Port, gpioData_Pin)==1);

    uint8_t i;
    for (i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(gpioSck_GPIO_Port, gpioSck_Pin, GPIO_PIN_SET);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(gpioData_GPIO_Port, gpioData_Pin))
        {
            buffer ++;
        }

        HAL_GPIO_WritePin(gpioSck_GPIO_Port, gpioSck_Pin, GPIO_PIN_RESET);
    }

    for (i = 0; i < data.gain; i++)
    {
    	HAL_GPIO_WritePin(gpioSck_GPIO_Port, gpioSck_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(gpioSck_GPIO_Port, gpioSck_Pin, GPIO_PIN_RESET);
    }

    buffer = buffer ^ 0x800000;

    return buffer;
}

void HX711_Tare(uint8_t times)
{
    int sum = HX711_Average_Value(times);
    data.offset = sum;
}

void HX711_Calibration(int weight, int value){
	data.a = (float)(weight)/(value - data.offset);
	data.b = (float)(weight)/(value - data.offset)*data.offset;
}

int HX711_Value_Gram(){
	return ((data.a*HX711_Value()) - data.b);
}



