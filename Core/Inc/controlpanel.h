/*
 * controlpanel.h
 *
 *  Created on: 23.09.2019
 *      Author: Konto_U�ytkowe
 */

#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

/* Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "ethernetif.h"
#include "usb_device.h"
#include "tenso.h"

/* Task handlers */
TaskHandle_t xTaskSendDataHandle;
TaskHandle_t xTaskReceivedDataHandle;
TaskHandle_t xTaskBLDCHandle;
TaskHandle_t xTaskADCHandle;
TaskHandle_t xTaskTempF4UARTHandle;
TaskHandle_t xTaskTensoHandle;

/* Task functions prototypes */
void vTaskSendData(void *p);
void vTaskReceivedData(void *p);
void vTaskBLDC(void *p);
void vTaskADC(void *p);
void vTaskTempF4UART(void *p);
void vTaskTenso(void *p);

/* Variabes ETH */
struct netconn *conn, *newconn;
err_t err, accept_err;

/* Queue handlers */
QueueHandle_t xTempF4UARTQueue;

/* Semaphore handlers */
SemaphoreHandle_t xMutexSensValue;
SemaphoreHandle_t xMutexBLDC;

/* Typedef */
typedef struct {
	uint8_t num;
	uint8_t arg;
	uint8_t rise_time;
	uint8_t stay_time;
	uint8_t fall_time;
}CMD_MODE_t;


/* Macro */
#define STOP_COMMAND					0
#define START_COMMAND					1
#define CHANGE_DUTY_COMMAND				2
#define TENSO_OFFSET_COMMAND 			3
#define TENSO_CALIBRATION_COMMAND		4
#define LONG_TEST_COMMAND				5
#define SHORT_TEST_COMMAND				6
#define THROTTLE_TEST_COMMAND			7
#define START_PWM						8
#define STOP_PWM						9
#define PAUSE							10
#define CONTINUE						11

#define UARTF4_TEMP_FRAME_SIZE			50

#define DUTY_MIN						25
#define DUTY_MAX						95

#define VIBRO_TABLE_LEN					10
#define HAL_TABLE_LEN					10

/* Variables */
struct {
	char temp[UARTF4_TEMP_FRAME_SIZE+1];
	uint16_t tenso_value;
	uint16_t vibro_value[VIBRO_TABLE_LEN];
	uint16_t shunt_value;
	uint16_t hal_value[HAL_TABLE_LEN];
} sens_value;

char uartf4_received[UARTF4_TEMP_FRAME_SIZE+1];
char usr_msg[200];
uint8_t UART_command;
uint8_t str[100];

uint16_t duty;

float Temperature;
float Vsense;

uint32_t adcValue[3];

uint8_t reset_time;
char current_time[40];

uint8_t rise, stay, fall;


/* Function prototypes */
void extract_arg(CMD_MODE_t *cmd, char * received_command);
void usb_print(char * msg);
void uart_print(char * msg);
char * get_time(uint32_t start_ms);
void copy_table(uint16_t tab[], uint16_t tab_bufor[], uint8_t table_len);
void empty_table(uint16_t tab[], uint8_t table_len);
// ***********make frame*************//
char * add_zero_to_string(char * str, int jak_dlugosc);
char * int_to_stringv2(int liczba);
void insert_to_frame(char frame[], int index, char * element);
void make_framev2(char frame[], char * time, char * temp_f4, int vibro_value[10], int shunt_value, int hal_value[10], int tenso_value);


#endif /* CONTROLPANEL_H_ */
