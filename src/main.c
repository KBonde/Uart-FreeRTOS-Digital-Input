/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

//User tasks
void menu_task (void *pvArgs);
void read_task (void *pvArgs);

//Functions that help send/receive text through serial
void newLine();
void clrScr();
void printStr(char text_to_print[]);
void printNum(int number_to_print);
void readInput(char buffer_to_hold_text[], uint16_t size_of_input);

//Menu functions
void display_main_menu();
int send_choice(int choice_to_send);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define LED2 GPIO_PIN_5
#define USRBTN GPIO_PIN_13

/* USER CODE END 0 */

QueueHandle_t choice_queue = 0;
SemaphoreHandle_t choice_mutex = 0;

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  clrScr(); //Clear the screen when reset is pressed, just nice when restarting...

  xTaskCreate(menu_task, "Sender task", 1024, NULL, 2, NULL);
  xTaskCreate(read_task, "Receiver task", 1024, NULL, 1, NULL);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  choice_mutex = xSemaphoreCreateMutex();

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  choice_queue = xQueueCreate(1, sizeof(int));
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  vTaskStartScheduler();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

void menu_task(void *pvArgs) {

	char input_buff[1] = {0};

	for(;;) {
		if(xSemaphoreTake(choice_mutex, pdMS_TO_TICKS(500))) { //Only run when choice_mutex is available
			clrScr();

			display_main_menu();

			readInput(input_buff, 1); //Get users choice here
			printStr(input_buff);

			if(input_buff[0] == '0') { //Continuous read mode
				if(send_choice(0) != 0) printStr("Failed to enter continuous read mode");

			} else if(input_buff[0] == '1') { //Single read mode
				if(send_choice(1) != 0) printStr("Failed to enter single read mode");
			}

			else { //If invalid number is entered
				xSemaphoreGive(choice_mutex); //Give mutex, so task can grab it again (avoids deadlock)

			}
		}
	}
}

void read_task(void *pvArgs) {
	char input_buffer[1] = {0}; //Holds user input
	char user_input_total[100] = {0}; //Holds entire user input
	int i = 0; //Counter
	int choice_to_receive = 0; //Variable that holds the users input


	for(;;) {
		if(xSemaphoreTake(choice_mutex, pdMS_TO_TICKS(500))) {
			clrScr();

			if(!xQueueReceive(choice_queue, &choice_to_receive, pdMS_TO_TICKS(100))) //Check if queue sent correctly
				printStr("Failed to receive choice from menu task"); //Print error message if not

			memset(user_input_total, 0, 100); //Reset the 'string' that holds the entire user input

			if(choice_to_receive == 0) { //Continuous read mode

				printStr("You entered continuous read mode. Press '-' to stop continuous read mode and return to main menu");
				newLine();

				TickType_t xLastWake = 0; //
				const TickType_t xFrequency = pdMS_TO_TICKS(1000);


				while(1) {

					xLastWake = xTaskGetTickCount();

					if(!HAL_UART_Receive(&huart2, input_buffer, 1, (uint32_t)10)) {
						user_input_total[i] = input_buffer[0]; //Add last input to input array
						i++;
						printStr(input_buffer);
						newLine();
					}


					if(input_buffer[0] == '-') {

						user_input_total[i-1] = 0;

						newLine();
						printStr("You entered '");
						printStr(user_input_total);
						printStr("' in total.");
						newLine();
						printStr("Enter anything to continue...");

						char x[1]; //Temp var to store whatever is entered
						readInput(x, 1); //Just used to pause the program, so the user can see their input

						input_buffer[0] = 0; //Reset user input
						i = 0; //Reset counter

						break; //Return to main menu
					}

					vTaskDelayUntil(&xLastWake, xFrequency); //Only run task once per second

				}

			} else if(choice_to_receive == 1) { //Single read mode

				printStr("You entered single read mode. Press '-' to stop single read mode and return to main menu.");
				newLine();

				while(1) {
					readInput(input_buffer, 1);
					printStr(input_buffer);
					newLine();

					if(input_buffer[0] == '-') {
						newLine();
						printStr("You entered '");
						printStr(user_input_total);
						printStr("' in total.");
						newLine();
						printStr("Enter anything to continue...");

						char x[1]; //Temp var to store whatever is entered
						readInput(x, 1); //Just used to pause the program, so the user can see their input

						input_buffer[0] = 0; //Reset user input
						i = 0; //Reset counter

						break;
					}

					/*We add the input here to avoid adding the '-' to the array as well*/
					user_input_total[i] = input_buffer[0]; //Add last input to input array
					i++;

				}


			} else {
				printStr("Received invalid number, returning to main menu...");
				vTaskDelay(pdMS_TO_TICKS(5000)); //Delay so message stays on screen
			}

			xSemaphoreGive(choice_mutex); //Give mutex back to main menu
		} else {
			printStr("Did not receive choice_mutex, returning to main menu... ");
			vTaskDelay(pdMS_TO_TICKS(5000)); //Delay so message stays on screen
		}
	}
}

void display_main_menu() {

	clrScr();

	printStr("Enter a number to select an option");
	newLine();
	printStr("<0> for continues reading");
	newLine();
	printStr("<1> for single read");
	newLine();
	newLine();
	printStr("Enter here: ");
}

int send_choice(int choice_int) {
	/*Added different error codes - mainly to make debugging easier*/
	if(!xQueueSend(choice_queue, &choice_int, pdMS_TO_TICKS(100))) return -1;
	if(!xSemaphoreGive(choice_mutex)) return -2;
	vTaskDelay(pdMS_TO_TICKS(100));

	return 0;
}

//Functions to help display/read text easier
void newLine() { //Basically prints a new line and moves cursor all the way to the left
	HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);
}

void clrScr() { // Clears the screen. Moves cursor to 0,0
	HAL_UART_Transmit(&huart2, (uint8_t*)"\033[0;0H", strlen("\033[0;0H"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\033[2J", strlen("\033[2J"), HAL_MAX_DELAY);
}

void printStr(char str[]) { //Prints the string that is passed into it
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void printNum (int num) { //Prints number that is passed here (max of 20 in length atm)
	char buff[20];
	sprintf(buff, "%i", num);
	printStr(buff);
}

void readInput(char buff[], uint16_t size) { //Reads user input into buffer that is passed
	HAL_UART_Receive(&huart2, buff, size, HAL_MAX_DELAY);
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
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
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
