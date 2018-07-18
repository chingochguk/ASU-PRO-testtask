/**
  *  @file
  *  @brief Обработка сообщений для АСУ ПРО
  *
  *  Содержит функцию обработки сообщений UART и функцию оправки сообщения UART
  */
/**
  * @Author Свиридов Юрий Викторович, Челябинск, 07.2018
  */
/** 
  * @mainpage  
  *           Данный проект создан в программной среде IAR Embedded Workbench 6.30.
  *           Для базового конфигурирования микроконтроллера использовалось приложение STM32CubeMX.
  *           Документация создавалась в Doxygen. Микроконтроллер STM32F103C8T6.
  *           
  *      В проекте реализовано:
  *        1) Прием сообщений по UART. 
  *        2) Обработка сообщений оканчивающихся символом NULL.
  *        3) Выполнение команд содержащихся в сообщениях и отправка ответа. 
  *        4) Измерение напряжения с помощью АЦП с дискретностью 1000Гц.( Запуск
  *           начала измерения происходит в обработчике прерывания таймера, который и 
  *           задает частоту дискретизации).
  *        5) Считывание с помощью DMA значения АЦП и передача его в регистр сравнения
  *           таймера, который выдает сигнал ШИМ. Скважность сигнала ШИМ зависит от
  *           значения напряжения АЦП.(На микроконтроллере STM32F103C8T6 нет ЦАП, 
  *           поэтому используется этот вариант.)
  *        6) Подсчет среднего значения сигнала АЦП и вывод его в качестве ответа на команду. 
  *        7) Использование ОСРВ FreeRTOS.
  *        8) Использование 3-ех потоков (задач) ОС.(ReceiveTask, LedTask, MovingAverageTask)
  *        9) Использование очередей, семафоров.
  *
  *      Некоторые требования тестового задания не были выполнены так как,
  *       по мнению автора, не являются оптимальными.
  */

#include "asu_pro_parser.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


extern uint8_t  UART1_RxBuf[UART1_RX_BUFFER_SIZE];
extern uint16_t UART1_RxHead;
extern uint16_t UART1_RxTail;
extern osMessageQId UartRXQueueHandle;
extern osSemaphoreId TXSemHandle;



/**
*     Обработка входящих сообщений UART

*  @param[in] qdata.pos Позиция последнего символа сообщения в кольцевом буфере
*  @param[in] qdata.len  Длина сообщения 
*  @return идентификатор принятого сообщения (LED_OFF, LED_ON, GET_ADC_AVG_VOLTAGE, SET_ADC_SAMPLE_RATE) или ошибку (ERROR)
*/
uint32_t Parse_AsuProMess(UART_Queue_typedef qdata)
{
 uint32_t isOK = ERROR; ///< переменная результата 
 
  
  switch(UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - (qdata.len+1)) &
                                       UART1_RX_BUFFER_SIZE_MASK ]) ///< проверяем первый символ сообщения
  { 
      /// LED_OFF
      case 'L':
        if( qdata.len == 7)
        {
          
          if((UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 3) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'F')&&
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 4) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'O')&&
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 6) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'D')&& 
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 7) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'E')) ///< проверяем выборочно символы в сообщении
          {
            isOK = _LED_OFF;           
          }
        }
       /// LED_ON 
      else
        if(qdata.len == 6)
        {
          if((UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 3) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'O')&& 
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 5) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'D')&&
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 6) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'E')&& 
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 2) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'N'))
          {
            isOK = _LED_ON; 
          }
        }
      break;
      /// GET_ADC_AVG_VOLTAGE
      case 'G':        
        if(qdata.len < 19)
                      break;        
        if((UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 3) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'G')&& 
           (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 4) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'A')&&
           (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 18) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'T')&& 
           (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 12) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'A'))
         {
          isOK = _GET_ADC_AVG_VOLTAGE;
         }
      break;
      /// SET_ADC_SAMPLE_RATE
      case 'S':         
          if((UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 3) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'T')&& 
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 5) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'R')&&
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 10) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'M')&& 
             (UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - 12) &
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'S'))
        {
          isOK = _SET_ADC_SAMPLE_RATE;
        } 
     break;
     default:break;
  }
#ifdef isBUSY           
 UART1_RxTail = ((UART1_RxTail + qdata.len) & UART1_RX_BUFFER_SIZE_MASK);
#endif                          
 return  isOK;         
}


/**
*    Отправка сообщения по UART

*  @param[in] mes   Сообщение
*  @param[in] len   Длина сообщения           
*  @return Ошибка (ERROR), Успех (SUCCESS)
*/
uint32_t Send_AsuProMess(char *mes, uint16_t len)
{ 
 uint16_t i = 0;

  do
  {
    
    UART_PutChar(mes[i]);///< кладем символ в регистр данных DR 
    
    if( xSemaphoreTake(TXSemHandle, 20 / portTICK_RATE_MS) != pdTRUE )  ///< ждем пока сработает прерывание по окончании передачи
                                                               return ERROR;
    i++;
  }
  while(i<len);
      
 return SUCCESS;
}