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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "asu_pro_parser.h"
#include "string.h"
#include "ctype.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId ReceiveTaskHandle;
uint32_t ReceiveTaskBuffer[ 256 ];
osStaticThreadDef_t ReceiveTaskControlBlock;
osThreadId LedTaskHandle;
uint32_t LedTaskBuffer[ 128 ];
osStaticThreadDef_t LedTaskControlBlock;
osThreadId MovingAverageTaHandle;
uint32_t MovingAverageTaskBuffer[ 128 ];
osStaticThreadDef_t MovingAverageTaskControlBlock;
osMessageQId UartRXQueueHandle;
uint8_t UartRXQueueBuffer[ 1 * sizeof( uint32_t ) ];
osStaticMessageQDef_t UartRXQueueControlBlock;
osMessageQId LEDQueueHandle;
uint8_t LEDQueueBuffer[ 1 * sizeof( uint16_t ) ];
osStaticMessageQDef_t LEDQueueControlBlock;
osMessageQId ADCValQueueHandle;
uint8_t ADCValQueueBuffer[ 1 * sizeof( uint32_t ) ];
osStaticMessageQDef_t ADCValQueueControlBlock;
osMessageQId ADCAverQueueHandle;
uint8_t ADCAverQueueBuffer[ 1 * sizeof( float ) ];
osStaticMessageQDef_t ADCAverQueueControlBlock;
osSemaphoreId TXSemHandle;
osStaticSemaphoreDef_t TXSemControlBlock;
osSemaphoreId GetAvSemHandle;
osStaticSemaphoreDef_t GetAvSemControlBlock;

/* USER CODE BEGIN Variables */

#define ADC_VAL_BUFFER_SIZE                   128 ///< размер буфера для значений ADC
#define ADC_VAL_BUFFER_SIZE_MASK             ((ADC_VAL_BUFFER_SIZE) - 1)


#define  LED_FlachRate       (100  / portTICK_RATE_MS)///<5Гц
#define  LED_SLOW            (1000 / portTICK_RATE_MS)///<0.5Гц
#define  LED_MEDIUM          (500 / portTICK_RATE_MS) ///<1Гц
#define  LED_FAST            (250  / portTICK_RATE_MS)///<2Гц


#define  NOT_READY           0xFFFFFFFF ///< идентификатор не готовности среднего значения ADC

uint16_t ADC_ValBuf[ADC_VAL_BUFFER_SIZE];///< буфер для хранения значений ADC

//// набор состояний светодиода
enum flash_state{
  fl_off = 0,
  fl_on,
  fl_slow,
  fl_med,
  fl_fast
};
 

void LED_Flash(enum flash_state fl_state, portTickType *LED_SleepTime);
char * itoa(int value, char* str, int base);
void strreverse(char* begin, char* end);

extern osMessageQId UartQueueHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartReceiveTask(void const * argument);
void StartLedTask(void const * argument);
void StartMovingAverageTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of TXSem */
  osSemaphoreStaticDef(TXSem, &TXSemControlBlock);
  TXSemHandle = osSemaphoreCreate(osSemaphore(TXSem), 1);

  /* definition and creation of GetAvSem */
  osSemaphoreStaticDef(GetAvSem, &GetAvSemControlBlock);
  GetAvSemHandle = osSemaphoreCreate(osSemaphore(GetAvSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ReceiveTask */
  osThreadStaticDef(ReceiveTask, StartReceiveTask, osPriorityNormal, 0, 256, ReceiveTaskBuffer, &ReceiveTaskControlBlock);
  ReceiveTaskHandle = osThreadCreate(osThread(ReceiveTask), NULL);

  /* definition and creation of LedTask */
  osThreadStaticDef(LedTask, StartLedTask, osPriorityNormal, 0, 128, LedTaskBuffer, &LedTaskControlBlock);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of MovingAverageTa */
  osThreadStaticDef(MovingAverageTa, StartMovingAverageTask, osPriorityNormal, 0, 128, MovingAverageTaskBuffer, &MovingAverageTaskControlBlock);
  MovingAverageTaHandle = osThreadCreate(osThread(MovingAverageTa), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UartRXQueue */
  osMessageQStaticDef(UartRXQueue, 1, uint32_t, UartRXQueueBuffer, &UartRXQueueControlBlock);
  UartRXQueueHandle = osMessageCreate(osMessageQ(UartRXQueue), NULL);

  /* definition and creation of LEDQueue */
  osMessageQStaticDef(LEDQueue, 1, uint16_t, LEDQueueBuffer, &LEDQueueControlBlock);
  LEDQueueHandle = osMessageCreate(osMessageQ(LEDQueue), NULL);

  /* definition and creation of ADCValQueue */
  osMessageQStaticDef(ADCValQueue, 1, uint32_t, ADCValQueueBuffer, &ADCValQueueControlBlock);
  ADCValQueueHandle = osMessageCreate(osMessageQ(ADCValQueue), NULL);

  /* definition and creation of ADCAverQueue */
  osMessageQStaticDef(ADCAverQueue, 1, float, ADCAverQueueBuffer, &ADCAverQueueControlBlock);
  ADCAverQueueHandle = osMessageCreate(osMessageQ(ADCAverQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  /* USER CODE END RTOS_QUEUES */
}

/* StartReceiveTask function */
void StartReceiveTask(void const * argument)
{

  /* USER CODE BEGIN StartReceiveTask */
 portBASE_TYPE taskWoken;
 UART_Queue_typedef qdata;
 enum flash_state LED_State = fl_off;
 float ADC_Val;
 char temp_str1[10];
 char temp_str2[10];
  
  LL_USART_EnableIT_TC(USART1);///< активируем прерывания по приходу байта и по окончании передачи байта
  LL_USART_EnableIT_RXNE(USART1);///< здесь берем семафор один раз так как сработало прерывание TC
  xSemaphoreTake(TXSemHandle, 20 / portTICK_RATE_MS);
  //------------------------------
  /* Infinite loop */
  for(;;)
  {   
    if(xQueueReceiveFromISR( UartRXQueueHandle, &qdata, &taskWoken ) == pdTRUE)///< ждем сообщение по UART       
      
      switch (Parse_AsuProMess(qdata))///< парсим
      {       
       case ERROR: ///< если пришло не понятно что          
          LED_State = fl_med;///< частота светодиода 1HZ
          if(Send_AsuProMess("BUSY\r\n", 6) == ERROR)///< отправляем отклик 
            LED_State = fl_fast;         
          xQueueSendToBack( LEDQueueHandle, &LED_State, 0 ); ///<обновляем частоту моргания светодиода в потоке LedTask 
        break;
        
        case _LED_OFF:
          LED_State = fl_off;///< состояние светодиода - отключен
          if(Send_AsuProMess("OK\r\n", 4) == ERROR)
             LED_State = fl_fast; 
          xQueueSendToBack( LEDQueueHandle, &LED_State, 0 );
        break;
        
        case _LED_ON: 
          LED_State = fl_on; 
          if(Send_AsuProMess("OK\r\n", 4) == ERROR)
             LED_State = fl_fast;            
          xQueueSendToBack( LEDQueueHandle, &LED_State, 0 ); 
        break;
        
        case _GET_ADC_AVG_VOLTAGE:         
          if(xSemaphoreGive(GetAvSemHandle)== pdTRUE)///< посылаем запрос о необходимости среднего значения напряжения потоку MovingAverageTask
          {          
           if(xQueueReceive( ADCAverQueueHandle, &ADC_Val, 50 / portTICK_RATE_MS )== pdTRUE)///< получаем среднее значение напряжения
           {           
            if(ADC_Val == NOT_READY)///< если среднее значение не готово то сигнализируем 
            {
             if(Send_AsuProMess("NOT READY\r\n", 11) == ERROR)
               LED_State = fl_fast;          
            }
            else
            {
              itoa(ADC_Val, temp_str1, 10); ///< переводим в ASCII целую часть
              strcat(temp_str1, "."); ///< добавляем точку             
              uint16_t i = (ADC_Val - (int)ADC_Val) * 1000;///< берем дробную часть  
              itoa(i, temp_str2, 10);                  
              strcat(temp_str1, temp_str2);
              if(Send_AsuProMess(temp_str1, strlen(temp_str1))!= ERROR)///< ответ
              {
                Send_AsuProMess("\r\n",2);
                Send_AsuProMess("OK\r\n", 4);
              }
              else 
               LED_State = fl_fast; 
            }
           }
           xQueueSendToBack( LEDQueueHandle, &LED_State, 0 );
          }
        break;
        
        case _SET_ADC_SAMPLE_RATE:
          LL_TIM_SetAutoReload(TIM1, 1000);///< устанавливаем дискретезацию АЦП 10Гц
          if(Send_AsuProMess("Sample rate = 10HZ\r\n",20)!= ERROR)         
               Send_AsuProMess("OK\r\n", 4);
          else
            LED_State = fl_fast;
             xQueueSendToBack( LEDQueueHandle, &LED_State, 0 );
        break;
      }
  }
  /* USER CODE END StartReceiveTask */
}

/* StartLedTask function */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
portTickType LED_SleepTime = LED_FlachRate;  
enum flash_state LED_State = fl_off; 
uint32_t NowTicks;///< текущее значение тиков  
  /* Infinite loop */
  for(;;)
  {  
     xQueueReceive( LEDQueueHandle, &LED_State, 0 );///< проверяем очередь
     NowTicks = xTaskGetTickCount(); 
     LED_Flash(LED_State, &LED_SleepTime);
     vTaskDelayUntil(&NowTicks, LED_SleepTime); 
  }
  /* USER CODE END StartLedTask */
}

/* StartMovingAverageTask function */
void StartMovingAverageTask(void const * argument)
{
  /* USER CODE BEGIN StartMovingAverageTask */
 /**
*     MovingAverageTask
  
*  @details Используется алгоритм простого скользящнго среднего:
  
*           Есть кольцевой буфер ADC_ValBuf из ADC_VAL_BUFFER_SIZE значений. 
*           Есть сумма значений АЦП ADC_ValSum.
  
*           При появлении нового значения ADC_Val из суммы удаляется 
*           значение самого старого. К сумме прибавляется новое
*           и делится на ADC_VAL_BUFFER_SIZE
  */
 portBASE_TYPE taskWoken;
 uint32_t ADC_ValSum = 0;///< сумма значений АЦП
 uint16_t ADC_OldVal_pos = 0;///< индекс 
 uint32_t ADC_Val;///< значение АЦП
 uint8_t isFull = 0;///< флаг заполненности буфера
 float ADC_AverVal;
  /* Infinite loop */
  for(;;)
  {
   
   if(xQueueReceiveFromISR( ADCValQueueHandle, &ADC_Val, &taskWoken ) == pdTRUE) ///< новое значение АЦП
   {     
     if(isFull)///< проверяем флаг заполненности буфера
     {      
       ADC_ValSum = ADC_ValSum - ADC_ValBuf[ADC_OldVal_pos];///< вычитаем из суммы самое старое значение АЦП
       ADC_ValSum = ADC_ValSum + ADC_Val;///< прибавляем к сумме новое значение
       ADC_ValBuf[ADC_OldVal_pos] = (uint16_t)ADC_Val;///< добавляем в буфер новое значение вместо самого старого
       ADC_OldVal_pos++;///< сдвигаем "курсор" на позицию самого старого значения АЦП       
       ADC_OldVal_pos &= ADC_VAL_BUFFER_SIZE_MASK;///< вместо проверки на переполнение 
     }
     else
     {
       ADC_ValSum = ADC_ValSum + ADC_Val; 
       ADC_ValBuf[ADC_OldVal_pos] = (uint16_t)ADC_Val;
       ADC_OldVal_pos++;
       ADC_OldVal_pos &= ADC_VAL_BUFFER_SIZE_MASK;
       if(ADC_OldVal_pos == 0)
                        isFull = 1;
     }
    if(!isFull)
      ADC_AverVal = NOT_READY;
    else
      /** Разрядность АЦП 12 бит = 4096 уровней квантования
        * Разрешение АЦП 3,3/4096 = 0,0008057 = 800 мкВ   
        * @example Читаем число 2050 значит U = 2050*0,0008 = 1,64В*/
      ADC_AverVal = (ADC_ValSum/ADC_VAL_BUFFER_SIZE) * 0.0008;
    if(xSemaphoreTake(GetAvSemHandle, 20 / portTICK_RATE_MS)== pdTRUE)
     xQueueSendToBack( ADCAverQueueHandle, &ADC_AverVal, 10 / portTICK_RATE_MS );///< если есть запрос то отправляем среднее значение  
   }
       
  }
  /* USER CODE END StartMovingAverageTask */
}

/* USER CODE BEGIN Application */

void LED_Flash(enum flash_state fl_state, portTickType *LED_SleepTime)
{ 
  switch(fl_state)
  {
    case fl_off:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      *LED_SleepTime = LED_FlachRate;
    break;
    
    case fl_on:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      *LED_SleepTime = LED_FlachRate;
    break;

    case fl_slow:
       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
       *LED_SleepTime = LED_SLOW;
    break;
    
    case fl_med:
       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
       *LED_SleepTime = LED_MEDIUM;
    break;

    case fl_fast:
       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
       *LED_SleepTime = LED_FAST;
    break;
    
    default:break;
  }
}  

/*-------------------------------------------------------------------------------------------
  itoa * Ansi C "itoa" based on Kernighan & Ritchie's "Ansi C"
  with slight modification to optimize for specific architecture:
---------------------------------------------------------------------------------------------*/
char * itoa(int value, char* str, int base) {
	static char num[] = "0123456789ABCDEF";
	char* wstr=str;
	int sign;
	div_t res;
	// Validate base
	if (base<2 || base>16){ *wstr='\0'; return 0; }
	// Take care of sign
	if ((sign=value) < 0) value = -value;
	// Conversion. Number is reversed.
	do {
		res = div(value,base);
		*wstr++ = num[res.rem];
	}while(value=res.quot);
	if(sign<0) *wstr++='-';
	*wstr='\0';
	// Reverse string
	strreverse(str,wstr-1);
		return (str);
}

void strreverse(char* begin, char* end) 
{
	char aux;
	while(end>begin)
		aux=*end, *end--=*begin, *begin++=aux;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
