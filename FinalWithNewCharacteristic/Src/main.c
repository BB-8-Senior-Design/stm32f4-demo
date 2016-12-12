/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac2;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*####################################*\
 * AUDIO Private Variables
\*####################################*/
#define AUDIO_buffer_size 96000
uint8_t AUDIO_buffer [AUDIO_buffer_size];
volatile uint32_t AUDIO_buffer_out = 0;
volatile uint8_t AUDIO_current_file = 0;
volatile uint8_t AUDIO_previous_file = 1;
volatile uint32_t AUDIO_current_file_length = 60049;
uint8_t AUDIO_num_files = 4;
FATFS AUDIO_SD_disk;
FIL AUDIO_SD_file;
char AUDIO_SD_diskPath[4] = "\0\0\0\0";


uint8_t AUDIO_buffer_chunk_0 [8192];
uint8_t AUDIO_buffer_chunk_1 [8192];
uint8_t AUDIO_current_chunk_length = 8192;
volatile uint8_t AUDIO_current_buffer = 1;
volatile uint8_t AUDIO_file_opened = 0;


// DAC stuff
DAC_HandleTypeDef DacHandle;
static DAC_ChannelConfTypeDef sConfig;

/*####################################*\
 * Servo Private Variables
\*####################################*/

volatile uint8_t SERVO_direction = 0;
volatile uint16_t SERVO_dutyCycle = 1500; // servo duty cycle in microseconds

/*####################################*\
 * Motor Private Variables
\*####################################*/

volatile uint8_t MOTOR_direction = 0; // 0 is stop, 1 is go, lol rip in peace
volatile uint8_t prev_MOTOR_direction = 0;
volatile int arbitraryCounter = 0;

volatile int motorSpeed1 = 0;
volatile int motorSpeed2 = 0;

/*####################################*\
 * Bluetooth Private Variables
\*####################################*/
char bluetoothTxBuffer[100] = "R,1\r\n";
uint8_t bluetoothRxBuffer[2] = {0};

uint8_t commandBuffer[30] = {0};
uint8_t commandBufferIndex = 0;
volatile uint8_t commandReceiveCompleted = 0;

/*####################################*\
 * Battery gauge Private Variables
\*####################################*/
volatile uint32_t rawADC;
volatile float batteryPercentage;
volatile uint8_t batteryPercentageBluetooth = 100;
volatile uint8_t oldBatteryPercentageBluetooth = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM11_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/*####################################*\
 * Audio Function Prototypes
\*####################################*/
void AUDIO_SD_readFile(char * filename);
void AUDIO_SD_configure(void);
void AUDIO_DAC_configure(void);
void AUDIO_SD_readFileChunk(char * filename);
void AUDIO_DAC_configureChunk(void);


/*####################################*\
 * Bluetooth Function Prototypes
\*####################################*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/// Update battery service value
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
		rawADC = HAL_ADC_GetValue(hadc);
		batteryPercentage = ((((((float)rawADC)/4096.0)*13.3)-9.0)/3.4)*100;
		batteryPercentageBluetooth = (uint8_t) batteryPercentage;
		if (batteryPercentageBluetooth != oldBatteryPercentageBluetooth) {//((batteryPercentageBluetooth < (oldBatteryPercentageBluetooth-1)) || (batteryPercentageBluetooth > (oldBatteryPercentageBluetooth+1))) {
			// things have CHANGED
			oldBatteryPercentageBluetooth = batteryPercentageBluetooth;
			// otherwise things haven't
			uint8_t commandToSend[] = "SUW,2A19,00\r\n";
			char sixteensplace [2];
			char onesplace [2];
			sprintf(onesplace, "%1x", ((int)(batteryPercentageBluetooth%16)));
			sprintf(sixteensplace, "%1x", (uint8_t)(((batteryPercentageBluetooth-(batteryPercentageBluetooth%16))/16)));
			commandToSend[9] = sixteensplace[0];
			commandToSend[10] = onesplace[0];
			HAL_UART_Transmit(&huart2, commandToSend, 13, 100);
		}
	}
}


void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {
	if (huart->Instance == USART2) {
		// If we are receiving a new command, zero out its buffer
		if (commandBufferIndex == 0) for (uint8_t i = 0; i < 30; i++) commandBuffer[i] = 0;

		// Check if we have received new-line - end of command
		if (commandReceiveCompleted == 0 && bluetoothRxBuffer[0] != '\n') {
			// Add the received character to the end of the commandBuffer
			commandBuffer[commandBufferIndex++] = bluetoothRxBuffer[0];
		} else if (bluetoothRxBuffer[0] == '\n') {
			commandBufferIndex = 0;
			commandReceiveCompleted = 1;
		}
	}
	HAL_UART_Receive_IT(&huart2, bluetoothRxBuffer, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
	// HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	HAL_UART_Receive_IT(&huart2, bluetoothRxBuffer, 1);
}

void BLUE_Process_Command(uint8_t* command) {
	// We are only interested in the WV,0018 command
	if (command[0] == 'W' && command[1] == 'V') {
		if (command[5] == '1' && command[6] == '8') {
			GPIO_PinState stateToSet = GPIO_PIN_SET;
			if (command[8] == '0' && command[9] == '0') {
				// AUDIO_current_file = 0;
				stateToSet = GPIO_PIN_RESET;
			}
			if (command[8] == '0' && command[9] != '0') {
				AUDIO_current_file = command[9]-'0';
			}
			//HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, stateToSet);
		}
	}
	// The second characteristic is the WV,001A command
	// I guess I'll use this for the servo

	if (command[0] == 'W' && command[1] == 'V') {
		if (command[5] == '1' && command[6] == 'A') {
			//GPIO_PinState stateToSet = GPIO_PIN_SET;
			if (command[8] == '0' && command[9] == '0') {
				SERVO_direction = 0;
			}
			if (command[8] == '0' && command[9] == '1') {
				SERVO_direction = 1;
			}
			if (command[8] == '0' && command[9] == '2') {
				SERVO_direction = 2;
			}
		}
	}

	// WV, 001C for motor speed
	if (command[0] == 'W' && command[1] == 'V') {
		if (command[5] == '1' && command[6] == 'C') {
			if (command[8] == '0' && command[9] == '0') { // stop motors
				HAL_GPIO_WritePin(MC1_INH_3_3_GPIO_Port, MC1_INH_3_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MC2_INH_3_3_GPIO_Port, MC2_INH_3_3_Pin, GPIO_PIN_RESET);
				arbitraryCounter = 0;
				prev_MOTOR_direction = MOTOR_direction;
				MOTOR_direction = 0;

				// If we were going forwards, switch the motors to go backwards
				if (prev_MOTOR_direction == 1) {
					TIM4->CCR4 = TIM4->CCR1;
					TIM8->CCR2 = TIM8->CCR1;
					TIM4->CCR1 = 0;
					TIM8->CCR1 = 0;
				}
			} else if (command[8] == '0' && command[9] == '1') {
				HAL_GPIO_WritePin(MC1_INH_3_3_GPIO_Port, MC1_INH_3_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MC2_INH_3_3_GPIO_Port, MC2_INH_3_3_Pin, GPIO_PIN_SET);
				prev_MOTOR_direction = MOTOR_direction;
				MOTOR_direction = 1;
				// HAL_GPIO_WritePin(MC2_IN1_GPIO_Port, MC2_IN1_Pin, GPIO_PIN_SET);
				// HAL_GPIO_WritePin(MC2_IN2_GPIO_Port, MC2_IN2_Pin, GPIO_PIN_RESET);
			} else if (command[8] == '0' && command[9] == '2') {
				HAL_GPIO_WritePin(MC1_INH_3_3_GPIO_Port, MC1_INH_3_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MC2_INH_3_3_GPIO_Port, MC2_INH_3_3_Pin, GPIO_PIN_SET);
				prev_MOTOR_direction = MOTOR_direction;
				MOTOR_direction = 2;
			} else if (command[8] == '0' && command[9] == '3') {
				HAL_GPIO_WritePin(MC1_INH_3_3_GPIO_Port, MC1_INH_3_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MC2_INH_3_3_GPIO_Port, MC2_INH_3_3_Pin, GPIO_PIN_SET);
				prev_MOTOR_direction = MOTOR_direction;
				MOTOR_direction = 3;
			} else if (command[8] == '0' && command[9] == '4') {
				HAL_GPIO_WritePin(MC1_INH_3_3_GPIO_Port, MC1_INH_3_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MC2_INH_3_3_GPIO_Port, MC2_INH_3_3_Pin, GPIO_PIN_SET);
				prev_MOTOR_direction = MOTOR_direction;
				MOTOR_direction = 4;
			}
		}
	}

	// WV,001E for weird new motor speed characteristic
	if (command[0] == 'W' && command[1] && 'V') {
		if (command[5] == '1' && command[6] == 'E') {

			HAL_GPIO_WritePin(MC1_INH_3_3_GPIO_Port, MC1_INH_3_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MC2_INH_3_3_GPIO_Port, MC2_INH_3_3_Pin, GPIO_PIN_SET);

			volatile char motor1Speed[3] = {command[8], command[9], '\0'};
			volatile char motor2Speed[3] = {command[10], command[11], '\0'};

			motorSpeed1 = (int)strtol(motor1Speed, NULL, 16);
			motorSpeed2 = (int)strtol(motor2Speed, NULL, 16);
			if (motorSpeed1 > 127) {
				motorSpeed1 -= 256;
			}
			if (motorSpeed2 > 127) {
				motorSpeed2 -= 256;
			}
		}
	}
	commandReceiveCompleted = 0;
}


/*############################*\
 * Servo Timer Update Interrupt
\*############################*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
	uint8_t SERVO_speed = 50;
	if(htim->Instance == TIM2) {
		if(SERVO_direction == 0) {
					return;
		}
		if(SERVO_direction == 1) {
			// go right
			SERVO_dutyCycle += SERVO_speed;
			if (SERVO_dutyCycle > 2400) {
				SERVO_dutyCycle = 2400;
			}
		} else if (SERVO_direction == 2) {
			// go left
			SERVO_dutyCycle -= SERVO_speed;
			if (SERVO_dutyCycle < 600) {
				SERVO_dutyCycle = 600;
			}
		}
		TIM1->CCR1 = (uint32_t)SERVO_dutyCycle;
	} else if (htim->Instance == TIM13) {
		HAL_GPIO_TogglePin(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);
	} else if (htim->Instance == TIM4) {
		// Set the left motor
		if (motorSpeed1 < 0) { // Send TIM4 backwards
			uint8_t pwmMotorSpeed1 = 0 - motorSpeed1;
			TIM4->CCR4 = pwmMotorSpeed1;
			TIM4->CCR1 = 0;
		} else {
			uint8_t pwmMotorSpeed1 = motorSpeed1;
			TIM4->CCR1 = pwmMotorSpeed1;
			TIM4->CCR4 = 0;
		}

		// Set the right motor
		if (motorSpeed2 < 0) { // Send TIM8 backwards
			uint8_t pwmMotorSpeed2 = 0 - motorSpeed2;
			TIM8->CCR2 = pwmMotorSpeed2;
			TIM8->CCR1 = 0;
		} else {
			uint8_t pwmMotorSpeed2 = motorSpeed2;
			TIM8->CCR1 = pwmMotorSpeed2;
			TIM8->CCR2 = 0;
		}
		/*if (MOTOR_direction != 0) {
			arbitraryCounter++;
			if (arbitraryCounter > 100) {
				arbitraryCounter = 0;
				if (MOTOR_direction == 1) {
					// Start TIM4 Channel 1 and TIM8 Channel 1
					if (TIM4->CCR1 < 50) TIM4->CCR1 += 1;
					else TIM4->CCR1 = 50;

					if (TIM8->CCR1 < 50) TIM8->CCR1 += 1;
					else TIM8->CCR1 = 50;

					// Stop TIM4 Channel 4 and TIM8 Channel 2
					if (TIM4->CCR4 > 1) TIM4->CCR4 -= 1;
					else TIM4->CCR4 = 0;

					if (TIM8->CCR2 > 1) TIM8->CCR2 -= 1;
					else TIM8->CCR2 = 0;
				}

				if (MOTOR_direction == 2) {
					// Start TIM4 Channel 4 and TIM8 Channel 2
					if (TIM4->CCR4 < 50) TIM4->CCR4 += 1;
					else TIM4->CCR4 = 50;

					if (TIM8->CCR2 < 50) TIM8->CCR2 += 1;
					else TIM8->CCR2 = 50;

					// Stop TIM4 Channel 1 and TIM8 Channel 1
					if (TIM4->CCR1 > 1) TIM4->CCR1 -= 1;
					else TIM4->CCR1 = 0;

					if (TIM8->CCR1 > 1) TIM8->CCR1 -= 1;
					else TIM8->CCR1 = 0;
				}

				if (MOTOR_direction == 3) {
					// Start TIM4 Channel 4 and TIM8 Channel 1
					if (TIM4->CCR4 < 30) TIM4->CCR4 += 1;
					else TIM4->CCR4 = 30;

					if (TIM8->CCR1 < 30) TIM8->CCR1 += 1;
					else TIM8->CCR1 = 30;

					// Stop TIM4 Channel 1 and TIM8 Channel 2
					if (TIM4->CCR1 > 1) TIM4->CCR1 -= 1;
					else TIM4->CCR1 = 0;

					if (TIM8->CCR2 > 1) TIM8->CCR2 -= 1;
					else TIM8->CCR2 = 0;
				}

				if (MOTOR_direction == 4) {
					// Start TIM4 Channel 1 and TIM8 Channel 2
					if (TIM4->CCR1 < 30) TIM4->CCR1 += 1;
					else TIM4->CCR1 = 30;
					if (TIM8->CCR2 < 30) TIM8->CCR2 += 1;
					else TIM8->CCR2 = 30;

					// Stop TIM4 Channel 4 and TIM8 Channel 1
					if (TIM4->CCR4 > 1) TIM4->CCR4 -= 1;
					else TIM4->CCR4 = 0;

					if (TIM8->CCR1 > 1) TIM8->CCR1 -= 1;
					else TIM8->CCR1 = 0;
				}
			}
		} else {
			// all stop
			// TIM4->CCR1 = 0;
			// TIM4->CCR4 = 0;
			// TIM8->CCR1 = 0;
			// TIM8->CCR2 = 0;
			if (TIM4->CCR1 != 0 || TIM4->CCR4 != 0 || TIM8->CCR1 != 0 || TIM8->CCR2 != 0) {
				arbitraryCounter++;
				if (arbitraryCounter > 100) {
					arbitraryCounter = 0;

					if (TIM4->CCR1 > 1) TIM4->CCR1 -= 1;
					else TIM4->CCR1 = 0;

					if (TIM4->CCR4 > 1) TIM4->CCR4 -= 1;
					else TIM4->CCR4 = 0;

					if (TIM8->CCR1 > 1) TIM8->CCR1 -= 1;
					else TIM8->CCR1 = 0;

					if (TIM8->CCR2 > 1) TIM8->CCR2 -= 1;
					else TIM8->CCR2 = 0;
				}
			} else {
				arbitraryCounter = 0;
			}
		}
		*/
	} else if (htim->Instance == TIM11) {
		//adc
		HAL_ADC_Start_IT(&hadc1);
	}

}


/////////////////////////
// SD Stuff           //
////////////////////////

void AUDIO_SD_configure(void) {
	//if(FATFS_LinkDriver(&SD_Driver,AUDIO_SD_diskPath)==0) {
	// AUDIO_SD_diskPath[0] = '0';
	FRESULT mountResult = f_mount(&AUDIO_SD_disk, (TCHAR const*)AUDIO_SD_diskPath, 0);
	if(mountResult != FR_OK) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
		//cry
	}
	// HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	//}
	return;
}

void AUDIO_SD_readFile(char * filename) {
	UINT bytesRead;
	FRESULT openResult = f_open(&AUDIO_SD_file, filename, FA_OPEN_ALWAYS | FA_READ);
	if(openResult != FR_OK)
	{
	  /* 'STM32.TXT' file Open for write Error */
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	  return;
	}
	FRESULT readResult = f_read(&AUDIO_SD_file, &AUDIO_buffer, AUDIO_buffer_size, &bytesRead);
	if(readResult != FR_OK) {
	  // please read the file properly
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	  return;
	}
	f_close(&AUDIO_SD_file);
	AUDIO_current_file_length = bytesRead;
	return;
}

void AUDIO_DAC_configure(void) {
	if(HAL_DAC_Init(&DacHandle) != HAL_OK) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	HAL_DAC_Stop_DMA(&DacHandle, DAC_CHANNEL_2);
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if(HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		/* Channel configuration Error */
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	if((HAL_DAC_Start(&DacHandle,DAC_CHANNEL_2)) != HAL_OK) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	if((HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_2, (uint32_t*)AUDIO_buffer, AUDIO_current_file_length, DAC_ALIGN_8B_R) != HAL_OK)) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	// HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
}

void AUDIO_SD_readFileChunk(char * filename) {
	UINT bytesRead;
	uint8_t * currentBuffer;
	if (AUDIO_current_buffer == 0) {
		currentBuffer = AUDIO_buffer_chunk_1;
	} else {
		currentBuffer = AUDIO_buffer_chunk_0;
	}
	FRESULT openResult = FR_DISK_ERR;
	if (!AUDIO_file_opened) {
		openResult = f_open(&AUDIO_SD_file, filename, FA_OPEN_ALWAYS | FA_READ);
		if(openResult != FR_OK)
		{
		  /* 'STM32.TXT' file Open for write Error */
			// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		  return;
		}
		FRESULT readResult = f_read(&AUDIO_SD_file, AUDIO_buffer_chunk_0, 8192, &bytesRead);
		if(readResult != FR_OK) {
		  // please read the file properly
			// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		  return;
		}
		AUDIO_file_opened = 1;
	}
	FRESULT readResult = f_read(&AUDIO_SD_file, currentBuffer, 8192, &bytesRead);
	if(readResult != FR_OK) {
	  // please read the file properly
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	  return;
	}
	if(f_eof(&AUDIO_SD_file)) {
		f_close(&AUDIO_SD_file);
		AUDIO_current_file = 0;
	}
	AUDIO_current_chunk_length = bytesRead;
	return;
}

void AUDIO_DAC_configureChunk(void) {
	uint8_t * currentBuffer;
	if (AUDIO_current_buffer == 0) {
		currentBuffer = AUDIO_buffer_chunk_0;
	} else {
		currentBuffer = AUDIO_buffer_chunk_1;
	}
	if(HAL_DAC_Init(&DacHandle) != HAL_OK) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	HAL_DAC_Stop_DMA(&DacHandle, DAC_CHANNEL_2);
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if(HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		/* Channel configuration Error */
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	if((HAL_DAC_Start(&DacHandle,DAC_CHANNEL_2)) != HAL_OK) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	if((HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_2, (uint32_t*)currentBuffer, AUDIO_current_chunk_length, DAC_ALIGN_8B_R) != HAL_OK)) {
		// HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		return;
	}
	// HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
}

/*#############################*\
 * DAC Transfer complete callback
\*#############################*/


void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac)
{
	AUDIO_current_file = 0;
}

/*
void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac)
{
	AUDIO_DAC_configureChunk();
	AUDIO_current_buffer = (AUDIO_current_buffer + 1)%1;
	AUDIO_SD_readFileChunk("rip");
}
*/


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_FATFS_Init();
  MX_TIM13_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM11_Init();

  /* USER CODE BEGIN 2 */
  // Wake up the RN4020
  HAL_Delay(200);
  HAL_GPIO_WritePin(BLUE_WAKE_HW_GPIO_Port, BLUE_WAKE_HW_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(BLUE_WAKE_SW_GPIO_Port, BLUE_WAKE_SW_Pin, GPIO_PIN_SET);
  HAL_Delay(300);

  // Restart RN4020 module
  HAL_UART_Transmit(&huart2, (uint8_t *) bluetoothTxBuffer, 5, 10);

  // After restarting, will automatically advertise if configured with SR,20000000
  HAL_Delay(300);

  // Set UART receive on an interrupt
  HAL_UART_Receive_IT(&huart2, bluetoothRxBuffer, 1);


  // DAC Stuff
  DacHandle.Instance = DAC;
  AUDIO_SD_configure();
  //AUDIO_SD_readFile("1.bb8");
  HAL_TIM_Base_Start(&htim6);
  //AUDIO_DAC_configure();

  // Enable PWM

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  // Heartbeat LED
  HAL_TIM_Base_Start_IT(&htim13);

  //Battery status timers
  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start_IT(&htim11);


  // this doesn't exist
  HAL_GPIO_WritePin(SPEAKER_SHUTDOWN_GPIO_Port, SPEAKER_SHUTDOWN_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if (commandReceiveCompleted) BLUE_Process_Command(commandBuffer);
	  // if (changeBatteryService) BLUE_Update_Battery();
	  if (AUDIO_current_file != AUDIO_previous_file) {
		  AUDIO_previous_file = AUDIO_current_file;
		  char fileToRead [6] = {(AUDIO_current_file+'0'),'.','b','b','8'};
		  AUDIO_SD_readFile(fileToRead);
		  AUDIO_DAC_configure();
		  /* not working rip */
		  //AUDIO_file_opened = 0;
		  //f_close(&AUDIO_SD_file);
		  //AUDIO_current_buffer = 0;
		  //AUDIO_SD_readFileChunk(fileToRead);
		  //AUDIO_DAC_configureChunk();
	  }
	  if (AUDIO_current_file == 0) {
		  HAL_DAC_Stop_DMA(&DacHandle, DAC_CHANNEL_2);
		  HAL_DAC_Stop(&DacHandle,DAC_CHANNEL_2);
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 167;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1749;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 65535;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 12817;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 4800;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 499;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HEARTBEAT_LED_Pin|MC1_INH_3_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLUE_WAKE_SW_Pin|BLUE_WAKE_HW_Pin|MC2_INH_3_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLUE_CMD_Pin|SPEAKER_SHUTDOWN_Pin|DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HEARTBEAT_LED_Pin MC1_INH_3_3_Pin */
  GPIO_InitStruct.Pin = HEARTBEAT_LED_Pin|MC1_INH_3_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_WAKE_SW_Pin BLUE_WAKE_HW_Pin MC2_INH_3_3_Pin */
  GPIO_InitStruct.Pin = BLUE_WAKE_SW_Pin|BLUE_WAKE_HW_Pin|MC2_INH_3_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_CMD_Pin SPEAKER_SHUTDOWN_Pin DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_CMD_Pin|SPEAKER_SHUTDOWN_Pin|DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
