/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "TLC5940_RGB.h"
#include "math.h"
#include <stdlib.h>
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMessageQId message_q_Handle;

osPoolId  mpool_id;

osPoolDef(mpool_id, 16, Led_selection);


/* USER CODE END Variables */
osThreadId TheTask1Handle;
osThreadId TheTask2Handle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static inline void TLC5940_SetGSUpdateFlag(void) {
	__asm__ volatile ("" ::: "memory");
	gsUpdateFlag = 1;
}
/* USER CODE END FunctionPrototypes */

void StartTask1(void const * argument);
void StartTask2(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	osMessageQDef(message_q, 5, uint32_t); // Declare a message queue
	message_q_Handle = osMessageCreate(osMessageQ(message_q), NULL);

	mpool_id = osPoolCreate (osPool (mpool_id));

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of TheTask1 */
  osThreadDef(TheTask1, StartTask1, osPriorityNormal, 0, 128);
  TheTask1Handle = osThreadCreate(osThread(TheTask1), NULL);

  /* definition and creation of TheTask2 */
  osThreadDef(TheTask2, StartTask2, osPriorityIdle, 0, 256);
  TheTask2Handle = osThreadCreate(osThread(TheTask2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the TheTask1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void const * argument)
{

  /* USER CODE BEGIN StartTask1 */
  /* Infinite loop */

	HAL_TIM_Base_Start_IT(&htim2);

	//========================= START init

	for(int i = 0; i < 15; i++)
	{
		while(gsUpdateFlag)
		{
		}
		TLC5940_SetAllGS(0);
		TLC5940_SetGS(i, 4095);
		TLC5940_SetGSUpdateFlag();
		HAL_Delay(500);
//		printf("check\n");
	}

	//========================= END init

	int32_t value;
//	printf("Task1 start\n");

	int simulatetime = 0;
	int maxsimultime = 10000;

	Led_selection *mled__selection;

//	int heartbeat = 0;

  for(;;)
  {
    value = osSemaphoreWait (myBinarySem01Handle, 1);        // Wait 1ms for the free semaphore
    if (value > 0) {

    	if(simulatetime > maxsimultime)
    	{
    		uint32_t randomchannel = rand() % 3;
    		uint32_t randomstep = rand() % 11; // 0 - 10
    		uint32_t randomled = rand() % 5;
    		uint32_t randomdelays = (rand() % 100000) + 5000; // 5 - 100 sec
    		uint32_t turnoff = rand() % 10;

    		maxsimultime = randomdelays;

//    		printf("led: %i channel: %i steps:%i delay:%i\n", (int)randomled, (int)randomchannel, (int)randomstep, (int)maxsimultime);

			HAL_GPIO_TogglePin(indicator_GPIO_Port,indicator_Pin);

			mled__selection = osPoolAlloc(mpool_id);
			mled__selection->channel = randomchannel;
			mled__selection->ledindex = randomled;
			mled__selection->stepselect = randomstep;

			if(turnoff == 0)
			{
				mled__selection->stepselect = 99;
			}

			osMessagePut(message_q_Handle, (uint32_t)mled__selection, osWaitForever);

			simulatetime = 0;
    	}
    	simulatetime++;

//    	if(heartbeat > 1000)
//    	{
//    		HAL_GPIO_TogglePin(indicator_GPIO_Port,indicator_Pin);
//    		heartbeat = 0;
//    	}
//    	heartbeat++;

	  osSemaphoreRelease (myBinarySem01Handle);            // Return a token back to a semaphore
	}

  }
  /* USER CODE END StartTask1 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the TheTask2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void const * argument)
{
  /* USER CODE BEGIN StartTask2 */
  /* Infinite loop */
//	printf("Task2 start\n");
	Led_selection *recv_led__selection;
	osEvent evt;
//	double stepsize_opt[5] = {0.01, 0.01 ,0.01, 0.01, 0.01};

//	double stepsize_opt[5] = {0.01 ,0.001, 0.005, 0.0001, 0.00001};

	double stepsize_opt[11] = {0.01, 0.01, 0.001, 0.001, 0.001, 0.005, 0.005, 0.005, 0.0001, 0.0001, 0.00001};

	Led led[5];

	int ledupdate;
	for(int i = 0;i < 5;i++)
	{
		led[i].r_steps = stepsize_opt[9];
		led[i].g_steps = stepsize_opt[9];
		led[i].b_steps = stepsize_opt[9];

		led[i].r_angle = 0;
		led[i].b_angle = 0;
		led[i].g_angle = 0;

		led[i].rval = 0;
		led[i].gval = 0;
		led[i].bval = 0;
	}

//	int rval = 0;
//	int gval = 0;
//	int bval = 0;

  for(;;)
  {
	  osSemaphoreWait (myBinarySem01Handle, 1);

	  evt = osMessageGet(message_q_Handle, 1);
	  if (evt.status == osEventMessage) {
		  recv_led__selection = evt.value.p;

		  ledupdate = recv_led__selection->ledindex;

		  switch (recv_led__selection->channel)
		  {
		  case 0:
			  if((int)recv_led__selection->stepselect == 99)
			  {
				  led[ledupdate].r_steps = 0;
			  }
			  else
			  {
				  led[ledupdate].r_steps = stepsize_opt[(int)recv_led__selection->stepselect];
			  }
			  break;
		  case 1:
			  if((int)recv_led__selection->stepselect == 99)
			  {
				  led[ledupdate].g_steps = 0;
			  }
			  else
			  {
				  led[ledupdate].g_steps = stepsize_opt[(int)recv_led__selection->stepselect];
			  }
			  break;
		  case 2:
			  if((int)recv_led__selection->stepselect == 99)
			  {
				  led[ledupdate].b_steps = 0;
			  }
			  else
			  {
				  led[ledupdate].b_steps = stepsize_opt[(int)recv_led__selection->stepselect];
			  }
			  break;
		  }

		  osPoolFree(mpool_id, recv_led__selection);
	  }

	for(int i = 0; i < 5;i++)
	{
	  led[i].r_angle = led[i].r_angle + led[i].r_steps;
	  led[i].g_angle = led[i].g_angle + led[i].g_steps;
	  led[i].b_angle = led[i].b_angle + led[i].b_steps;

	  if(led[i].r_angle > 6 * PI || led[i].r_steps == 0)
		{
		  led[i].r_angle = 0;
		}
	  if(led[i].g_angle > 6 * PI || led[i].g_steps == 0)
		{
		  led[i].g_angle = 0;
		}
	  if(led[i].b_angle > 6 * PI || led[i].b_steps == 0)
		{
		  led[i].b_angle = 0;
		}

//		set_led(i, (int)( (0.5*(1+sin(led[i].r_angle))) * 4095),
//				  (int)( (0.5*(1+sin(led[i].g_angle))) * 4095),
//				  (int)( (0.5*(1+sin(led[i].b_angle))) * 4095));

//	  rval = (int)( (0.5*(1+sin(sin(led[i].r_angle)))) * 4095);
//	  gval = (int)( (0.5*(1+sin(sin(led[i].g_angle)))) * 4095);
//	  bval = (int)( (0.5*(1+sin(sin(led[i].b_angle)))) * 4095);


	  led[i].rval = (int)( (0.5*(1+sin(led[i].r_angle))) * 4095);
	  led[i].gval = (int)( (0.5*(1+sin(led[i].g_angle))) * 4095);
	  led[i].bval = (int)( (0.5*(1+sin(led[i].b_angle))) * 4095);


	  while(gsUpdateFlag)
	  {
	  }
	  TLC5940_SetAllGS(0);
	  set_led(i, led[i].rval,  led[i].gval, led[i].bval);

	  TLC5940_SetGSUpdateFlag();
	}

	  osSemaphoreRelease (myBinarySem01Handle);
  }
  /* USER CODE END StartTask2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
