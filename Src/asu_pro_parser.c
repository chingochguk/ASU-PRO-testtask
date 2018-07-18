/**
  *  @file
  *  @brief ��������� ��������� ��� ��� ���
  *
  *  �������� ������� ��������� ��������� UART � ������� ������� ��������� UART
  */
/**
  * @Author �������� ���� ����������, ���������, 07.2018
  */
/** 
  * @mainpage  
  *           ������ ������ ������ � ����������� ����� IAR Embedded Workbench 6.30.
  *           ��� �������� ���������������� ���������������� �������������� ���������� STM32CubeMX.
  *           ������������ ����������� � Doxygen. ��������������� STM32F103C8T6.
  *           
  *      � ������� �����������:
  *        1) ����� ��������� �� UART. 
  *        2) ��������� ��������� �������������� �������� NULL.
  *        3) ���������� ������ ������������ � ���������� � �������� ������. 
  *        4) ��������� ���������� � ������� ��� � ������������� 1000��.( ������
  *           ������ ��������� ���������� � ����������� ���������� �������, ������� � 
  *           ������ ������� �������������).
  *        5) ���������� � ������� DMA �������� ��� � �������� ��� � ������� ���������
  *           �������, ������� ������ ������ ���. ���������� ������� ��� ������� ��
  *           �������� ���������� ���.(�� ���������������� STM32F103C8T6 ��� ���, 
  *           ������� ������������ ���� �������.)
  *        6) ������� �������� �������� ������� ��� � ����� ��� � �������� ������ �� �������. 
  *        7) ������������� ���� FreeRTOS.
  *        8) ������������� 3-�� ������� (�����) ��.(ReceiveTask, LedTask, MovingAverageTask)
  *        9) ������������� ��������, ���������.
  *
  *      ��������� ���������� ��������� ������� �� ���� ��������� ��� ���,
  *       �� ������ ������, �� �������� ������������.
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
*     ��������� �������� ��������� UART

*  @param[in] qdata.pos ������� ���������� ������� ��������� � ��������� ������
*  @param[in] qdata.len  ����� ��������� 
*  @return ������������� ��������� ��������� (LED_OFF, LED_ON, GET_ADC_AVG_VOLTAGE, SET_ADC_SAMPLE_RATE) ��� ������ (ERROR)
*/
uint32_t Parse_AsuProMess(UART_Queue_typedef qdata)
{
 uint32_t isOK = ERROR; ///< ���������� ���������� 
 
  
  switch(UART1_RxBuf[((UART1_RX_BUFFER_SIZE + qdata.pos) - (qdata.len+1)) &
                                       UART1_RX_BUFFER_SIZE_MASK ]) ///< ��������� ������ ������ ���������
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
                                       UART1_RX_BUFFER_SIZE_MASK ] == 'E')) ///< ��������� ��������� ������� � ���������
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
*    �������� ��������� �� UART

*  @param[in] mes   ���������
*  @param[in] len   ����� ���������           
*  @return ������ (ERROR), ����� (SUCCESS)
*/
uint32_t Send_AsuProMess(char *mes, uint16_t len)
{ 
 uint16_t i = 0;

  do
  {
    
    UART_PutChar(mes[i]);///< ������ ������ � ������� ������ DR 
    
    if( xSemaphoreTake(TXSemHandle, 20 / portTICK_RATE_MS) != pdTRUE )  ///< ���� ���� ��������� ���������� �� ��������� ��������
                                                               return ERROR;
    i++;
  }
  while(i<len);
      
 return SUCCESS;
}