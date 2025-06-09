/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
//For CAN
//For CAN1
CAN_RxHeaderTypeDef CAN1_RxHeader;
uint8_t CAN1_RxData[8];
uint8_t CAN1_TxData0[8], CAN1_TxData1[8], CAN1_TxData2[8];
CAN_TxHeaderTypeDef CAN1_TxHeader0, CAN1_TxHeader1, CAN1_TxHeader2;
//For CAN2
CAN_RxHeaderTypeDef CAN2_RxHeader;
CAN_TxHeaderTypeDef CAN2_TxHeader;
CAN_FilterTypeDef BrakePressure_filter0, BrakePressure_filter1, MPPT_filter,
		BMS_filter0, BMS_filter1, IMU_filter0, IMU_filter1;
uint8_t CAN2_RxData[8];
uint8_t Start_TxData[2] = { 0x0001, 0x0000 };
uint32_t CAN1_TxMailbox0, CAN1_TxMailbox1, CAN1_TxMailbox2;

uint16_t speed;
//For brake pressure sensor
uint8_t brake_pressure_status = 0;
int brake_pressure;

//For IMU
int32_t Pos_Ch1;
int32_t Pos_Ch2;
int16_t speed_Ch1;
int16_t speed_Ch2;
int16_t IMU_temp;
int16_t roll;
int16_t pitch;
uint8_t IMU_status = 0;
int16_t accX;
int16_t accY;
int16_t accZ;

//For cockpit temperature
int raw_temp;
float cockpit_temperature;

//For BMS
//uint8_t charging_cable_status;
//uint8_t batteryPack_charging_status;
//uint8_t batteryPack_powerLoss_status;
//uint8_t batteryPack_ready_status;
//uint8_t discharge_contactor_status;
//uint8_t charging_contactor_status;
int percentSOC = 0;						//
int16_t totalCurrent = 0;					//
uint16_t packVoltage = 0;					//Divide by 100
uint8_t failure_level_status;			//
int errorCode;							//
uint16_t highest_cell_voltage;			//Divide by 100
uint16_t lowest_cell_voltage;			//Divide by 100
uint8_t max_cell_temp;						//
uint8_t min_cell_temp;						//
//uint16_t max_allowable_dis_current;

//For MPPTs
//For MPPT1
float MPPTL_IP_voltage;
float MPPTL_IP_current;
float MPPTL_OP_voltage;
float MPPTL_OP_current;
float MPPTL_MOSFET_temp;				//1 int
float MPPTL_Controller_temp;			//1 int
uint8_t MPPTL_Rx_Error_counter;
uint8_t MPPTL_Tx_Error_counter;
uint8_t MPPTL_Tx_Overflow_counter;
uint8_t MPPTL_error_flag;				//1
uint8_t MPPTL_limit_flag;
uint8_t MPPTL_mode;						//1
uint8_t MPPTL_time_counter;

//For MPPT2
float MPPTR_IP_voltage;
float MPPTR_IP_current;
float MPPTR_OP_voltage;
float MPPTR_OP_current;
float MPPTR_MOSFET_temp;				//1	int
float MPPTR_Controller_temp;			//1	int
uint8_t MPPTR_Rx_Error_counter;
uint8_t MPPTR_Tx_Error_counter;
uint8_t MPPTR_Tx_Overflow_counter;
uint8_t MPPTR_error_flag;				//1
uint8_t MPPTR_limit_flag;				//1
uint8_t MPPTR_mode;
uint8_t MPPTR_time_counter;

//For ADCs
//For ADC1
int Voltage_Current_Sensor_ADC[2];
float aux_voltage;
int Voltage_Sensor_R1 = 10000;
int Voltage_Sensor_R2 = 1000;
float aux_current;

//For ADC2
int Thermistor_ADC[2];
float a = 0.000968178520318206;
float b = 0.000259752726776477;
float c = -1.562941382957E-09;

//For Thermistor1
int T1_R1 = 99400;
float T1_R2;
float T1_voltage;
float Left_thermistor_K;
float Left_thermistor_C;

//For Thermistor2
int T2_R1 = 99500;
float T2_R2;
float T2_voltage;
float Right_thermistor_K;
float Right_thermistor_C;

//For RPM
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;  // cm

//For Display
const unsigned char display_buff[8];
char i2cdata[2];
float RxData;
uint8_t Cmd_End[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
//For Display
void NEXTION_SendString(char *obj, char *str);
void NEXTION_SendInt(char *obj, int num);
void NEXTION_SendFloat(char *obj, float num, int dp);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (RxData == 0x02) {
		HAL_UART_Transmit(&huart4, (uint8_t*) "page 0", 6, 100);
		HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CAN1 receive interrupt
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_RxHeader, CAN1_RxData);
//	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_RxHeader, CAN2_RxData);
}
//CAN2 receive interrupt
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &CAN2_RxHeader, CAN2_RxData);
}
//For Display
	uint8_t Rx_Data[7];
	uint8_t Cmd_End[3] = { 0xff, 0xff, 0xff };
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
		//CAN1 Start and interrupt start
		HAL_CAN_Start(&hcan1);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

		//CAN2 Start and interrupt start
		HAL_CAN_Start(&hcan2);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

		//For Brake Pressure sensor
		CAN1_TxHeader0.DLC = 2;
		CAN1_TxHeader0.IDE = CAN_ID_STD;
		CAN1_TxHeader0.RTR = CAN_RTR_DATA;
		CAN1_TxHeader0.StdId = 0x0000;

//CAN Filters

//CAN1
//For MPPTs
		MPPT_filter.FilterActivation = ENABLE;
		MPPT_filter.FilterBank = 0;
		MPPT_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
		MPPT_filter.FilterMode = CAN_FILTERMODE_IDMASK;
		MPPT_filter.FilterIdHigh = 0x0600 << 5;
		MPPT_filter.FilterIdLow = 0;
		MPPT_filter.FilterMaskIdHigh = 0x0F00 << 5;
		MPPT_filter.FilterMaskIdLow = 0;
		MPPT_filter.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan1, &MPPT_filter);

//For brake pressure sensor
		BrakePressure_filter0.FilterActivation = ENABLE;
		BrakePressure_filter0.FilterBank = 1;
		BrakePressure_filter0.FilterFIFOAssignment = CAN_RX_FIFO0;
		BrakePressure_filter0.FilterIdHigh = 0x0181 << 5;
		BrakePressure_filter0.FilterIdLow = 0x0000;
		BrakePressure_filter0.FilterMaskIdHigh = 0x0000;
		BrakePressure_filter0.FilterMaskIdLow = 0x0000;
		BrakePressure_filter0.FilterMode = CAN_FILTERMODE_IDLIST;
		BrakePressure_filter0.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan1, &BrakePressure_filter0);

		BrakePressure_filter1.FilterActivation = ENABLE;
		BrakePressure_filter1.FilterBank = 2;
		BrakePressure_filter1.FilterFIFOAssignment = CAN_RX_FIFO0;
		BrakePressure_filter1.FilterIdHigh = 0x0701 << 5;
		BrakePressure_filter1.FilterIdLow = 0x0000;
		BrakePressure_filter1.FilterMaskIdHigh = 0x0000;
		BrakePressure_filter1.FilterMaskIdLow = 0x0000;
		BrakePressure_filter1.FilterMode = CAN_FILTERMODE_IDLIST;
		BrakePressure_filter1.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan1, &BrakePressure_filter1);

//CAN2
//For IMU
		IMU_filter0.FilterActivation = ENABLE;
		IMU_filter0.FilterBank = 16;
		IMU_filter0.FilterFIFOAssignment = CAN_RX_FIFO1;
		IMU_filter0.FilterIdHigh = 0x01FF << 5;
		IMU_filter0.FilterIdLow = 0x0000;
		IMU_filter0.FilterMaskIdHigh = 0;
		IMU_filter0.FilterMaskIdLow = 0;
		IMU_filter0.SlaveStartFilterBank = 14;
		IMU_filter0.FilterMode = CAN_FILTERMODE_IDLIST;
		IMU_filter0.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan2, &IMU_filter0);

		IMU_filter1.FilterActivation = ENABLE;
		IMU_filter1.FilterBank = 17;
		IMU_filter1.FilterFIFOAssignment = CAN_RX_FIFO1;
		IMU_filter1.FilterIdHigh = 0x02FF << 5;
		IMU_filter1.FilterIdLow = 0x0000;
		IMU_filter1.FilterMaskIdHigh = 0;
		IMU_filter1.FilterMaskIdLow = 0;
		IMU_filter1.SlaveStartFilterBank = 14;
		IMU_filter1.FilterMode = CAN_FILTERMODE_IDLIST;
		IMU_filter1.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan2, &IMU_filter1);
//For BMS
//BMS basic info message0
		BMS_filter0.FilterActivation = ENABLE;
		BMS_filter0.FilterBank = 14;
		BMS_filter0.FilterFIFOAssignment = CAN_RX_FIFO1;
		BMS_filter0.FilterMode = CAN_FILTERMODE_IDLIST;
		BMS_filter0.FilterIdHigh = (0x18FF28F4 >> 13);
		BMS_filter0.FilterIdLow = (0x18FF28F4 << 3) | 4;
		BMS_filter0.FilterMaskIdHigh = 0;
		BMS_filter0.FilterMaskIdLow = 0;
		BMS_filter0.SlaveStartFilterBank = 14;
		BMS_filter0.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan2, &BMS_filter0);

//BMS basic info message1
		BMS_filter1.FilterActivation = ENABLE;
		BMS_filter1.FilterBank = 15;
		BMS_filter1.FilterFIFOAssignment = CAN_RX_FIFO1;
		BMS_filter1.FilterMode = CAN_FILTERMODE_IDLIST;
		BMS_filter1.FilterIdHigh = (0x18FE28F4 >> 13);
		BMS_filter1.FilterIdLow = (0x18FE28F4 << 3) | 4;
		BMS_filter1.FilterMaskIdHigh = 0;
		BMS_filter1.FilterMaskIdLow = 0;
		BMS_filter1.SlaveStartFilterBank = 14;
		BMS_filter1.FilterScale = CAN_FILTERSCALE_32BIT;
		HAL_CAN_ConfigFilter(&hcan2, &BMS_filter1);

		//For display
		void NEXTION_SendInt(char *obj, int num) {
			char buffer[30];
			int len = snprintf(buffer, sizeof(buffer), "%s.val=%d", obj, num);
			HAL_UART_Transmit(&huart4, (uint8_t*) buffer, len, 100);
			HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
		}

		void NEXTION_SendFloat(char *obj, float num, int dp) {
			char buffer[30];
			int len = snprintf(buffer, sizeof(buffer), "%s.vvs1=%d", obj, dp);
			HAL_UART_Transmit(&huart4, (uint8_t*) buffer, len, 100);
			HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);

			int32_t number = (int32_t) (num * pow(10, dp));
			//		len = snprintf(buffer, sizeof(buffer), "%s.val=%d", obj, number);
			len = snprintf(buffer, sizeof(buffer), "%s.val=%ld", obj, number);
			HAL_UART_Transmit(&huart4, (uint8_t*) buffer, len, 100);
			HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
		}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			//For RPM ultrasonic
			HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_SET); // Set trigger HIGH
			__HAL_TIM_SET_COUNTER(&htim1, 0); // Reset the timer counter
			while (__HAL_TIM_GET_COUNTER(&htim1) < 10)
				;  // Wait for 10 µs
			HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_RESET); // Set trigger LOW

			// Wait for echo HIGH (start of echo)
			pMillis = HAL_GetTick();
			while (!(HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin))
					&& (HAL_GetTick() - pMillis) < 10)
				;
			Value1 = __HAL_TIM_GET_COUNTER(&htim1); // Record time of echo HIGH

			// Wait for echo LOW (end of echo)
			pMillis = HAL_GetTick();
			while ((HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin))
					&& (HAL_GetTick() - pMillis) < 50)
				;
			Value2 = __HAL_TIM_GET_COUNTER(&htim1); // Record time of echo LOW

			// Calculate distance
			if (Value2 > Value1) { // Check for valid reading
				uint32_t time_diff = Value2 - Value1; // Time difference in µs
				Distance = (time_diff * 0.034) / 2; // Distance in cm
			} else {
				Distance = 0; // Invalid reading, reset distance
			}

			//ADC1
			//Voltage Sensor
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			Voltage_Current_Sensor_ADC[0] = HAL_ADC_GetValue(&hadc1);
			aux_voltage = Voltage_Current_Sensor_ADC[0]
					* ((Voltage_Sensor_R1 + Voltage_Sensor_R2)
							/ Voltage_Sensor_R2) * 3.3 / 4095;

			//Current Sensor
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			Voltage_Current_Sensor_ADC[1] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			//ADC2
			//Thermistor1
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 100);
			Thermistor_ADC[0] = HAL_ADC_GetValue(&hadc2);
			T1_voltage = Thermistor_ADC[0] * 3.3 / 4095;
			T1_R2 = (T1_voltage * T1_R1) / (12 - T1_voltage);
			Left_thermistor_K = 1
					/ (a + b * log(T1_R2)
							+ c * log(T1_R2) * log(T1_R2) * log(T1_R2));
			Left_thermistor_C = Left_thermistor_K - 273.15;

			//Thermistor2
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 100);
			Thermistor_ADC[1] = HAL_ADC_GetValue(&hadc2);
			T2_voltage = Thermistor_ADC[0] * 3.3 / 4095;
			T2_R2 = (T2_voltage * T2_R1) / (12 - T2_voltage);
			Right_thermistor_K = 1
					/ (a + b * log(T2_R2)
							+ c * log(T2_R2) * log(T2_R2) * log(T2_R2));
			Right_thermistor_C = Right_thermistor_K - 273.15;
			HAL_ADC_Stop(&hadc2);

			HAL_CAN_AddTxMessage(&hcan2, &CAN1_TxHeader0, Start_TxData,
					&CAN1_TxMailbox0);

			switch (CAN1_RxHeader.StdId) {
//Brake pressure sensor
			case 0x0181:
				brake_pressure = CAN1_RxData[0] | CAN1_RxData[1] << 8
						| CAN1_RxData[2] << 16 | CAN1_RxData[3] << 24;
				break;
			case 0x0701:
				brake_pressure_status = 1;
				break;
//MPPTL
			case 0x0630:
				memcpy(&MPPTL_IP_current, CAN1_RxData, sizeof(float));
				memcpy(&MPPTL_IP_voltage, &CAN1_RxData[4], sizeof(float));
				break;
			case 0x0631:
				memcpy(&MPPTL_OP_current, CAN1_RxData, sizeof(float));
				memcpy(&MPPTL_OP_voltage, &CAN1_RxData[4], sizeof(float));
				break;
			case 0x0632:
				memcpy(&MPPTL_MOSFET_temp, CAN1_RxData, sizeof(float));
				memcpy(&MPPTL_Controller_temp, &CAN1_RxData[4], sizeof(float));
				break;
			case 0x0635:
				MPPTL_Rx_Error_counter = CAN1_RxData[0];
				MPPTL_Tx_Error_counter = CAN1_RxData[1];
				MPPTL_Tx_Overflow_counter = CAN1_RxData[2];
				MPPTL_error_flag = CAN1_RxData[3];
				MPPTL_limit_flag = CAN1_RxData[4];
				MPPTL_mode = CAN1_RxData[5];
				MPPTL_time_counter = CAN1_RxData[7];
				break;
//MPPTR
			case 0x0640:
				memcpy(&MPPTR_IP_current, CAN1_RxData, sizeof(float));
				memcpy(&MPPTR_IP_voltage, &CAN1_RxData[4], sizeof(float));
				break;
			case 0x0641:
				memcpy(&MPPTR_OP_current, CAN1_RxData, sizeof(float));
				memcpy(&MPPTR_OP_voltage, &CAN1_RxData[4], sizeof(float));
				break;
			case 0x0642:
				memcpy(&MPPTR_MOSFET_temp, CAN1_RxData, sizeof(float));
				memcpy(&MPPTR_Controller_temp, &CAN1_RxData[4], sizeof(float));
				break;
			case 0x0645:
				MPPTR_Rx_Error_counter = CAN1_RxData[0];
				MPPTR_Tx_Error_counter = CAN1_RxData[1];
				MPPTR_Tx_Overflow_counter = CAN1_RxData[2];
				MPPTR_error_flag = CAN1_RxData[3];
				MPPTR_limit_flag = CAN1_RxData[4];
				MPPTR_mode = CAN1_RxData[5];
				MPPTR_time_counter = CAN1_RxData[7];
				break;
			}
//BMS
			switch(CAN2_RxHeader.StdId){
				//For IMU
							case 0x077F:
								IMU_status=1;
							break;
				//Channel 1 speed and position
							case 0x01FF:
								Pos_Ch1 = (int16_t)(CAN1_RxData[0] | CAN1_RxData[1] << 8
								| CAN1_RxData[2] << 16 | CAN1_RxData[3] << 24);
								speed_Ch1 = (int16_t) (CAN1_RxData[4] | CAN1_RxData[5] << 8);
							break;
//				//Channel 2 speed and position
//							case 0x02FF:
//								Pos_Ch2 = (int16_t)(CAN1_RxData[0] | CAN1_RxData[1] << 8
//										| CAN1_RxData[2] << 16 | CAN1_RxData[3] << 24);
//								speed_Ch2 = (int16_t)(CAN1_RxData[4] | CAN1_RxData[5] << 8);
//							break;
//				Temperature, roll and pitch
							case 0x02FF:
								IMU_temp = (int16_t)(CAN1_RxData[0]);
								roll = (int16_t)(CAN1_RxData[1] | CAN1_RxData[2] << 8);
								pitch = (int16_t)(CAN1_RxData[3] | CAN1_RxData[4] << 8);
							break;
//				//Accelerometer
//							case 0x0380:
//								accX = (int16_t)(CAN1_RxData[0] | CAN1_RxData[1]<<8);
//								accY = (int16_t)(CAN1_RxData[2] | CAN1_RxData[3] << 8);
//								accZ = (int16_t)(CAN1_RxData[4] | CAN1_RxData[5] << 8);
//							break;
				}

			switch (CAN2_RxHeader.ExtId) {
			case 0x18FF28F4:
				percentSOC = CAN2_RxData[1];
				totalCurrent = (((int16_t) ((CAN2_RxData[3] >> 8)
						| CAN2_RxData[2])) * 0.1) - 500;
				packVoltage = ((CAN2_RxData[5] >> 8) | CAN2_RxData[4]) * 0.1;
				failure_level_status = CAN2_RxData[6];
				errorCode = CAN2_RxData[7];
			case 0x018FE28F4:
				lowest_cell_voltage = CAN2_RxData[0] | CAN2_RxData[1] >> 8;
				highest_cell_voltage = CAN2_RxData[2] | CAN2_RxData[3] >> 8;
				max_cell_temp = (CAN2_RxData[4] | CAN2_RxData[5] >> 8) - 40;
				min_cell_temp = (CAN2_RxData[6] | CAN2_RxData[7] >> 8) - 40;
			}

			if (HAL_I2C_IsDeviceReady(&hi2c3, 0xB5, 1, 100)) {
				HAL_I2C_Mem_Read(&hi2c3, 0xB4, 0x07, 1, (uint8_t*) i2cdata, 2,
						100);
				raw_temp = ((i2cdata[1] << 8) | (i2cdata[0]));
				cockpit_temperature = (raw_temp * 0.02) - 273.15;
			}

			if (brake_pressure > 0) {
				HAL_GPIO_WritePin(BL_MOSFET_GPIO_Port, BL_MOSFET_Pin,
						GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(BL_MOSFET_GPIO_Port, BL_MOSFET_Pin,
						GPIO_PIN_RESET);
			}
			CAN1_TxData0[0] = Left_thermistor_C;
			CAN1_TxData0[1] = Right_thermistor_C;
			CAN1_TxData0[2] = (int) (aux_voltage * 10);
			CAN1_TxData0[3] = (int) (aux_current * 10);
			CAN1_TxData0[4] = percentSOC;
			CAN1_TxData0[5] = (uint8_t) (totalCurrent);
			CAN1_TxData0[6] = (uint8_t) (packVoltage / 1000);
			CAN1_TxData0[7] = (uint8_t) errorCode;
//CAN Messages for telemetry
//		HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader1, CAN1_TxData0, &CAN1_TxMailbox1);
			CAN1_TxData1[0] = lowest_cell_voltage / 100;
			CAN1_TxData1[1] = highest_cell_voltage / 100;
			CAN1_TxData1[2] = failure_level_status;
			CAN1_TxData1[3] = errorCode;
//			CAN1_TxData1[4] = max_cell_temp;
			CAN1_TxData1[5] = min_cell_temp;
			CAN1_TxData1[6] = 0;
			CAN1_TxData1[7] = 0;
//For Display
			NEXTION_SendFloat("x0", Left_thermistor_C, 2);
			NEXTION_SendFloat("x1", totalCurrent, 2);
			NEXTION_SendFloat("x2", MPPTL_MOSFET_temp, 2);
			NEXTION_SendFloat("x5", Right_thermistor_C, 2);
			NEXTION_SendFloat("x7", MPPTR_MOSFET_temp, 2);
//			NEXTION_SendFloat("x8", max_cell_temp, 2);
			NEXTION_SendFloat("x6", packVoltage, 2);
			NEXTION_SendInt("n0", speed);
			NEXTION_SendInt("n1", percentSOC);
			HAL_Delay(100);
		}
	}

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN2_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN2_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = ENABLE;
  hadc2.Init.NbrOfDiscConversion = 1;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 15;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BL_MOSFET_GPIO_Port, BL_MOSFET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : trigger_Pin */
  GPIO_InitStruct.Pin = trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BL_MOSFET_Pin */
  GPIO_InitStruct.Pin = BL_MOSFET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BL_MOSFET_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
		}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
