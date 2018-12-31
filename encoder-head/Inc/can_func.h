#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f1xx_hal.h"
#include "main.h"

void callback(CanRxMsgTypeDef* pRxMsg);
void union_init();


typedef union{
   char ch[8];
   double in;
}char8_to_double;
   
extern char8_to_double can_RX_data;
extern char8_to_double can_TX_data; 

   
 #ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */