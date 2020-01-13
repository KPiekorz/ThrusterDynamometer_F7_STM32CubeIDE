/*
 * controlpanel.c
 *
 *  Created on: 23.09.2019
 *      Author: Konto_U�ytkowe
 */

#include "controlpanel.h"


extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

static char received_command[100];

const float V25 = 0.76; // [Volts]
const float Avg_slope = 0.025; //[Volts/degree]
const float SupplyVoltage = 3.0; // [Volts]
const float ADCResolution = 4095.0;

static uint32_t count_adc = 0;

static uint16_t vibro_value_table[VIBRO_TABLE_LEN];

static uint16_t hal_value_table[HAL_TABLE_LEN];

static uint32_t pausa_time;
static uint32_t start_ms;


/*
 * task do wysylania danych z czujnikow
 */
void vTaskSendData(void *p){

	char frame[150];
//	uint16_t frame_length = 0;
	uint32_t ulNotifiedValue = 0x0;
	start_ms = 0;
	pausa_time = 0;

	if(xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1){

		empty_table(sens_value.vibro_value, VIBRO_TABLE_LEN);
		empty_table(sens_value.hal_value, HAL_TABLE_LEN);
		sens_value.shunt_value = 0;
		sens_value.tenso_value = 0;
		strncpy(sens_value.temp, "", sizeof(sens_value.temp));

		xSemaphoreGive(xMutexSensValue);
	}

	reset_time = 1;


	while(1){

		if( xTaskNotifyWait( 0xFFFFFFFF, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE ){

			if(xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1){

				memset(frame, '\0', sizeof(frame));

				if (reset_time == 1){
					start_ms = HAL_GetTick();
					reset_time = 0;
				}else if( pausa_time > 0) {
						start_ms = HAL_GetTick() - pausa_time;
						pausa_time = 0;
				}


			    make_framev2(frame, get_time(start_ms), sens_value.temp,
			    		(int*)sens_value.vibro_value,
						sens_value.shunt_value,
						(int*)sens_value.hal_value,
						sens_value.tenso_value);



				if (accept_err == ERR_OK) {

					netconn_write(newconn, (const unsigned char* )frame, strlen(frame), NETCONN_COPY);

				}

				empty_table(sens_value.vibro_value, VIBRO_TABLE_LEN);
				empty_table(sens_value.hal_value, HAL_TABLE_LEN);
				sens_value.shunt_value = 0;

				xSemaphoreGive(xMutexSensValue);
			}

		}else{



		}

	}

}

/*
 * Przetworzenie odbebranych danych i wlaczenie odpowiedniej funkcji
 */
void vTaskReceivedData(void *p){

	CMD_MODE_t *command_mode;

	struct netbuf* buffer;
	void* data;
	u16_t len;
	err_t recv_err;

	vTaskSuspend(NULL);

	while(1){



			//Obsluga polaczenia
			if (accept_err == ERR_OK) {

				while ((recv_err = netconn_recv(newconn, &buffer)) == ERR_OK) {

					do {

						netbuf_data(buffer, &data, &len);

					} while (netbuf_next(buffer) >= 0);

					netbuf_delete(buffer);

					command_mode = (CMD_MODE_t*) pvPortMalloc(sizeof(CMD_MODE_t));
					extract_arg(command_mode, data);

					uart_print(data);

				switch (command_mode->num) {

				case START_COMMAND: {

					HAL_ADC_Start_DMA(&hadc1, adcValue, 3);

					HAL_TIM_Base_Start_IT(&htim11); // send
	//				HAL_TIM_Base_Start_IT(&htim9); // tenso

					HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);

					//pwm
	//				duty = DUTY_MIN;
	//				HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, &duty, 1);

					break;
				}
				case STOP_COMMAND: {

					HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);


					HAL_ADC_Stop_DMA(&hadc1);

	//				HAL_TIM_Base_Stop_IT(&htim9);
					reset_time = 1; HAL_TIM_Base_Stop_IT(&htim11);

					HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);

					if(xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1){

						empty_table(sens_value.vibro_value, VIBRO_TABLE_LEN);
						empty_table(sens_value.hal_value, HAL_TABLE_LEN);
						sens_value.shunt_value = 0;
						sens_value.tenso_value = 0;
						strncpy(sens_value.temp, "", sizeof(sens_value.temp));

						xSemaphoreGive(xMutexSensValue);
					}

					break;
				}
				case CHANGE_DUTY_COMMAND: {
					duty = command_mode->arg;
					break;
				}
				case TENSO_OFFSET_COMMAND: {
					HX711_Tare(10);
					break;
				}
				case TENSO_CALIBRATION_COMMAND: {
					HX711_Calibration(command_mode->arg, HX711_Average_Value(10));
					break;
				}
				case SHORT_TEST_COMMAND: {

					if(xSemaphoreTake(xMutexBLDC, 0) == 1){

						rise = command_mode->rise_time;
						stay = command_mode->stay_time;
						fall = command_mode->fall_time;

						xTaskNotify(xTaskBLDCHandle, 0x1, eSetValueWithoutOverwrite);

						xSemaphoreGive(xMutexBLDC);
					}

					break;
				}
				case LONG_TEST_COMMAND: {

					if(xSemaphoreTake(xMutexBLDC, 0) == 1){

						rise = command_mode->rise_time;
						fall = command_mode->fall_time;

						xTaskNotify(xTaskBLDCHandle, 0x2, eSetValueWithoutOverwrite);

						xSemaphoreGive(xMutexBLDC);
					}

					break;
				}
				case THROTTLE_TEST_COMMAND: {

					if(xSemaphoreTake(xMutexBLDC, 0) == 1){

						rise = command_mode->rise_time;
						fall = command_mode->fall_time;

						xTaskNotify(xTaskBLDCHandle, 0x3, eSetValueWithoutOverwrite);

						xSemaphoreGive(xMutexBLDC);
					}

					break;
				}
				case START_PWM: {

					duty = 0;
					HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t*)&duty, 1);

					break;
				}
				case STOP_PWM: {
					HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
					break;
				}
				case PAUSE: {

					// zatrzymuje pomiar z czujnikow, silnik jedzie dalej

					HAL_ADC_Stop_DMA(&hadc1);
					HAL_TIM_Base_Stop(&htim2);

					HAL_TIM_Base_Stop_IT(&htim9);
					HAL_TIM_Base_Stop_IT(&htim11);

					pausa_time = (HAL_GetTick() - start_ms);


					break;
				}
				case CONTINUE: {

					HAL_TIM_Base_Start(&htim2); // adc
					HAL_ADC_Start_DMA(&hadc1, adcValue, 3);

					HAL_TIM_Base_Start_IT(&htim11); // send
					HAL_TIM_Base_Start_IT(&htim9); // tenso


					break;
				}
				default: {

					break;
				}

				}

				vPortFree(command_mode);

				}

				netconn_close(newconn);
				netconn_delete(newconn);

			}

		accept_err = netconn_accept(conn, &newconn);

	}

}

/*
 * Odczyt danych z czujnikow wykorzystujacych ADC
 */
void vTaskADC(void *p){

	uint32_t ulNotifiedValue = 0x0;


	while(1){

		if( xTaskNotifyWait( 0xFFFFFFFF, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE ){

			/*
			 * Odczyt z wibracji co 1ms, z hal i shunt co 100 ms, w rytm timer�w, ktore beda
			 * uzywac funkcji xtasknotify()
			 */

			if(xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1){

				sens_value.shunt_value = adcValue[1];
				copy_table(vibro_value_table, sens_value.vibro_value, VIBRO_TABLE_LEN);		// vibro - adc1_ch3 - 10000Hz
				copy_table(hal_value_table, sens_value.hal_value, HAL_TABLE_LEN);			// hal - adc1_ch6	  - 10000Hz

				HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

				xSemaphoreGive(xMutexSensValue);
			}

		}else{

		}

	}

}

/*
 * Obsluga automatycznych trybow pracy hamowni
 */
void vTaskBLDC(void *p){

	uint32_t ulNotifiedValue = 0x0;
	uint16_t inc;

	while(1){


		if( xTaskNotifyWait( 0xFFFFFFFF, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE ){

			if(xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1){

			duty = DUTY_MIN;

			switch(ulNotifiedValue) {
			case 1: { // short

				inc = (DUTY_MAX - DUTY_MIN)/(rise - 1);
				for(int i = 0; i <= rise; i++){
					vTaskDelay(1000);
					duty += inc;
				}
				vTaskDelay(stay*1000);
				inc = (DUTY_MAX - DUTY_MIN)/(fall - 1);
				for(int i = 0; i <= fall; i++){
					vTaskDelay(1000);
					duty -= inc;
				}

				break;
			}
			case 2: { // long

				inc = (DUTY_MAX - DUTY_MIN)/(rise - 1);
				for(int i = 0; i <= rise; i++){
					vTaskDelay(1000);
					duty += inc;
				}
				vTaskDelay(60000);
				inc = (DUTY_MAX - DUTY_MIN)/(fall - 1);
				for(int i = 0; i <= fall; i++){
					vTaskDelay(1000);
					duty -= inc;
				}

				break;
			}
			case 3: { // throttle

				inc = (DUTY_MAX - DUTY_MIN)/(rise - 1);
				for(int i = 0; i <= rise; i++){
					vTaskDelay(1000);
					duty += inc;
				}
				inc = (DUTY_MAX - DUTY_MIN)/(fall - 1);
				for(int i = 0; i <= fall; i++){
					vTaskDelay(1000);
					duty -= inc;
				}

				break;
			}
			}

			xSemaphoreGive(xMutexSensValue);
			}

		}else{

		}

	}

}



/*
 * Odbior danych z temperatura od F4
 */
void vTaskTempF4UART(void *p){

	uint32_t ulNotifiedValue = 0x0;

	while(1){

		if( xTaskNotifyWait( 0xFFFFFFFF, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE ){

			if (xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1) {

				strcpy(sens_value.temp, uartf4_received);

				HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

				xSemaphoreGive(xMutexSensValue);
			}

		}else{

		}

	}

}


/*
 * Odbior i odczyt danych z tensometru
 */
void vTaskTenso(void *p){

	uint32_t ulNotifiedValue = 0x0;

	while(1){


		if( xTaskNotifyWait( 0xFFFFFFFF, 0, &ulNotifiedValue, portMAX_DELAY) == pdTRUE ){


			if (xSemaphoreTake(xMutexSensValue, portMAX_DELAY) == 1) {

//				sens_value.tenso_value = HX711_Value_Gram();

				xSemaphoreGive(xMutexSensValue);
			}

		}else{

		}

	}

}


/*******************Callback functions - USB and USART2**********************/
void USB_ReceivedCallBack(uint8_t *buf, uint32_t len){

	BaseType_t checkIfYieldRequired  = pdFALSE, xResult;;

	memset(received_command, '\0', sizeof(received_command));
	sprintf(received_command, "%s", buf);

	xResult = xTaskNotifyFromISR(xTaskReceivedDataHandle, 0, eNoAction, &checkIfYieldRequired);

	configASSERT( xResult == pdPASS );
	portYIELD_FROM_ISR(checkIfYieldRequired);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){  // function (callback) - receiving from stm32 f4

	BaseType_t checkIfYieldRequired  = pdFALSE, xResult;

	if (huart->Instance == USART2) {

		xResult = xTaskNotifyFromISR(xTaskTempF4UARTHandle, 0, eNoAction, &checkIfYieldRequired);

		HAL_UART_Receive_DMA(&huart2, (uint8_t*) uartf4_received, UARTF4_TEMP_FRAME_SIZE);

		configASSERT( xResult == pdPASS );
		portYIELD_FROM_ISR(checkIfYieldRequired);

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // tim2 trigger adc1 with 10000 Hz
{
	BaseType_t checkIfYieldRequired = pdFALSE, xResult;

	count_adc++;

	if (count_adc == 1) {
		for (int i = 0; i < VIBRO_TABLE_LEN; i++) {
			vibro_value_table[i] = 0;
			hal_value_table[i] = 0;
		}
	}

	//	vibro_value_table[count_adc-1] = adcValue[0];
	//	hal_value_table[count_adc-1] = adcValue[2];

	vibro_value_table[count_adc - 1]++;
	hal_value_table[count_adc - 1]++;

	if (count_adc == 5) {
		xResult = xTaskNotifyFromISR(xTaskADCHandle, 0, eNoAction, &checkIfYieldRequired);

		count_adc = 0;

		configASSERT(xResult == pdPASS);
		portYIELD_FROM_ISR(checkIfYieldRequired);
	}
}

/****************************Additionals functions*****************************/
void extract_arg(CMD_MODE_t *cmd, char * received_command){

	char argument[4];

	memset(argument, '\0', sizeof(argument));
	uint8_t i = 1; uint8_t j = 0;
	while( received_command[i] != '|') {

		argument[j] = received_command[i];
		j++;
		i++;

	}
	argument[j] = '\0';
	sscanf(argument, "%d", (int*)&((cmd->num))); // numer komendy

	memset(argument, '\0', sizeof(argument));
	j = 0; i++;
	while (received_command[i] != '|') {

		argument[j] = received_command[i];
		j++;
		i++;

	}
	argument[j] = '\0';
	sscanf(argument, "%d", (int*)&(cmd->arg)); // duty lub tenso kalibracja

	memset(argument, '\0', sizeof(argument));
	j = 0; i++;
	while (received_command[i] != '|') {

		argument[j] = received_command[i];
		j++;
		i++;

	}
	argument[j] = '\0';
	sscanf(argument, "%d", (int*)&(cmd->rise_time)); // rise time

	memset(argument, '\0', sizeof(argument));
	j = 0; i++;
	while (received_command[i] != '|') {

		argument[j] = received_command[i];
		j++;
		i++;

	}
	argument[j] = '\0';
	sscanf(argument, "%d", (int*)&(cmd->stay_time)); // stay time

	memset(argument, '\0', sizeof(argument));
	j = 0; i++;
	while (received_command[i] != ']') {

		argument[j] = received_command[i];
		j++;
		i++;

	}
	argument[j] = '\0';
	sscanf(argument, "%d", (int*)&(cmd->fall_time)); // fall time

}

//void usb_print(char * msg){
//	char msg_to_send[100];
//	memset(msg_to_send, '\0', sizeof(msg_to_send));
//	sprintf(msg_to_send, "%s\r\n", msg);
//	CDC_Transmit_FS(msg_to_send, strlen(msg_to_send));
//}

void uart_print(char * msg){
	char msg_to_send[100];
	sprintf(msg_to_send, "%s\r\n", msg);
	HAL_UART_Transmit(&huart3, (uint8_t *) msg_to_send, strlen(msg_to_send), HAL_MAX_DELAY);
}

char * get_time(uint32_t start_ms){
	uint16_t min, sec, milisec;
	uint32_t sys_tick_ms = HAL_GetTick() - start_ms;


	milisec = sys_tick_ms % 1000;
	sec = (sys_tick_ms / 1000);
	min = sec / 60;
	sec = (sys_tick_ms / 1000) % 60;

	strncpy(current_time, "", sizeof(current_time));
	sprintf(current_time, "%d|%d|%d", min, sec, milisec);
	return current_time;
}

void copy_table(uint16_t tab[], uint16_t tab_bufor[], uint8_t table_len){

	for(int i = 0; i < table_len; i++){
		tab_bufor[i] = tab[i];
	}

}

void empty_table(uint16_t tab[], uint8_t table_len){

	for(int i = 0; i < table_len; i++){
		tab[i] = 0;
	}

}



// ********************** make frame *********************************//
char * add_zero_to_string(char * str, int jak_dlugosc){
    if(strlen(str)>=jak_dlugosc) return str;

    int ile_zer = jak_dlugosc-strlen(str);
    char * new_str = (char*)malloc((strlen(str)+ile_zer+1)* sizeof(char));
    char * zero_string = (char*)malloc((ile_zer+1)* sizeof(char));

    int i = 0;
    zero_string[0] = '\0';
    while (i<ile_zer){
        zero_string[i++] = '0';
    }
    zero_string[i] = '\0';

    new_str[0] = '\0';   // ensures the memory is an empty string
    strcat(new_str, zero_string);
    strcat(new_str, str);

    free(zero_string);

    return new_str;
}

char * int_to_stringv2(int liczba){ // 0 - 9999

    if(liczba == 0) return "0";

    char * string_liczba = (char*)malloc(5* sizeof(char));
    int dzielnik = 1000;
    int int_length = 4;

    while(liczba/dzielnik == 0){
        liczba %=1000;
        dzielnik /=10;
        int_length--;
    }

    int i = 0;
    while (liczba>0 || int_length>0){
        string_liczba[i++] = (liczba/dzielnik)+48;
        liczba %= dzielnik;
        dzielnik /= 10;
        int_length--;
    }
    string_liczba[i] = '\0';

    return  string_liczba;
}

void insert_to_frame(char frame[], int index, char * element){
    int i;
    int j = 0;
    for(i = index; i < (index+strlen(element));i++){
        frame[i] = element[j++];
    }
    frame[i] = '\0';
}

void make_framev2(char frame[], char * time, char * temp_f4, int vibro_value[10], int shunt_value, int hal_value[10], int tenso_value){
    int i;
    char * string_value;
    insert_to_frame(frame, 0, "[");

    // time
    time = add_zero_to_string(time, 11);
    insert_to_frame(frame, strlen(frame), time);
    insert_to_frame(frame, strlen(frame), "|");

    // temp_f4
    temp_f4 = add_zero_to_string(temp_f4, 18);
    insert_to_frame(frame, strlen(frame), temp_f4);
    insert_to_frame(frame, strlen(frame), "|");

    // vibro
    for(i = 0; i < 10; i++){
        string_value = add_zero_to_string(int_to_stringv2(vibro_value[i]), 4);
        insert_to_frame(frame, strlen(frame), string_value);
        insert_to_frame(frame, strlen(frame), "|");
    }

    // shunt
    string_value = add_zero_to_string(int_to_stringv2(shunt_value), 4);
    insert_to_frame(frame, strlen(frame), string_value);
    insert_to_frame(frame, strlen(frame), "|");

    // hal
    for(i = 0; i < 10; i++){
        string_value = add_zero_to_string(int_to_stringv2(hal_value[i]), 4);
        insert_to_frame(frame, strlen(frame), string_value);
        insert_to_frame(frame, strlen(frame), "|");
    }

    // tenso
    string_value = add_zero_to_string(int_to_stringv2(tenso_value), 4);
    insert_to_frame(frame, strlen(frame), string_value);
    insert_to_frame(frame, strlen(frame), "]");
}







