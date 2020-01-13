/*
 * tenso.h
 *
 *  Created on: 23.09.2019
 *      Author: Konto_U¿ytkowe
 */

#ifndef TENSO_H_
#define TENSO_H_

/* Includes */
#include "main.h"


/* Macros */
#define DATA_GAIN_FACTOR	128


/* Typedef */
typedef struct _hx711 // definition of struct to control HX711 amplifiter
{
	float a; // parametr to calculate value from tensometer
	float b; // parametr to calculate value from tensometer
	int offset;
	int gain;
	// 1: channel A, gain factor 128
	// 2: channel B, gain factor 32
    // 3: channel A, gain factor 64
} HX711;


/* Function prototypes */
void HX711_Tare(uint8_t times); // tare tensometer - obtain offset value
int HX711_Value(); // obtain raw value from tensometer
int HX711_Average_Value(uint8_t times); // obtain average value form "times"-measurement
void HX711_Calibration(int weight, int value); // obtain value of parameters a and b - count tensometer value
int HX711_Value_Gram(); // function returns value from tensometer in grams


#endif /* TENSO_H_ */
