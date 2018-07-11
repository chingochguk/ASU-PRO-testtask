#ifndef ASU_PRO_PARSER_H
#define ASU_PRO_PARSER_H

#include "stdint.h"

//// ����� ��������������� ���������
enum Parser_state{
      _LED_OFF = 1,
      _LED_ON,
      _GET_ADC_AVG_VOLTAGE,
      _SET_ADC_SAMPLE_RATE
};

/**
    ��������� ������� ��������� ��������� UART
*/
typedef struct
{
  
  uint16_t len;///< �����
  uint16_t  pos;///< ������� ����� ���������
  
} UART_Queue_typedef;

uint32_t Parse_AsuProMess(UART_Queue_typedef qdata);
uint32_t  Send_AsuProMess(char *mes, uint16_t len);

#endif /* ASU_PRO_PARSER_H */