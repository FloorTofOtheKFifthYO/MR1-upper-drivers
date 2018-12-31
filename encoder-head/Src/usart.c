/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <stdarg.h>
#include "cmd.h"
#include <string.h>

uint8_t c_recv;    
uint8_t aRxBuffer[RXBUFFERSIZE];           //hal库使用串口接收缓冲
char USART_RX_BUF[USART_REC_LEN];       //自定义接收存放的数组
uint16_t USART_RX_STA=0;                   //接收状态标志，接收到0x0d，0x0a结束	


/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);  

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
    
  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
     hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);   
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */
    
  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */
    
  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */



/***************串口打印函数***************/
char uart_buffer[100 + 1];
void uprintf(char *fmt, ...)
{
  int size;
  
  va_list arg_ptr;
  
  va_start(arg_ptr, fmt);  
  
  size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
  va_end(arg_ptr);
  HAL_UART_Transmit(&huart1,(uint8_t *)uart_buffer,size,1000);
}

/*********************串口中断回调函数********************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
  if(huart->Instance==USART1){ 
    //USART_RX_STA的bit15为1，即2^15=32768,十六进制表示为0x8000，位与为0说明第十五位不为1，接收未完成
    if((USART_RX_STA&0x8000)==0)
    {
      //USART_RX_STA的bit14为1，即2^14=16384,十六进制表示为0X4000，位与不为0说明第十四位为1，接收到0x0d
      if((USART_RX_STA&0x4000)!=0)
      {
        if(aRxBuffer[0]!=0x0a)  USART_RX_STA=0;//接收到了0x0d但是缓冲区不是0x0a，则不符合我们的协议，接受错误重新开始
        else USART_RX_STA|=0x8000;//如果接收成功把USART_RX_STA按位或0x8000，把他的第15位置1表示接受完成
      }
      else// 未接收到0x0d
      {
        if(aRxBuffer[0]==0x0d)//缓冲区已经是0x0d的话
        {
          USART_RX_STA|=0x4000;//把USART_RX_STA的14位置1表示接收到0x0d
          USART_RX_BUF[USART_RX_STA&0x3fff]=aRxBuffer[0];
        }
        else//如果缓冲区还不是0x0d
        {
          USART_RX_BUF[USART_RX_STA&0x3fff]=aRxBuffer[0];//存储数组的索引为USART_RX_STA的0~13位，所以取出他们就位与001111111111，然后把缓冲区的值赋值给存储数组
          USART_RX_STA++;//存储一次标志加1，用于后面给出接收字符长度
          if(USART_RX_STA>(USART_REC_LEN-1))//宏定义接收最大长度USART_REC_LEN若接收的字符长度过长，则报错重新开始接收
            USART_RX_STA=0;//重新开始接收
        }       
      }                         
    }
  }
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);  
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  
  //__HAL_UART_CLEAR_OREFLAG(&huart1);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);  
  
}

void usart_exc()
{
  int cmd_argc,len;
  int erro_n;
  if(USART_RX_STA&0x8000){//检测是否接受完成  //接受完一次指令
    len=USART_RX_STA&0x3fff;//取出接收的长度
    if(len == 0){
      HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
      USART_RX_STA=0;
      return;
    }
    erro_n = cmd_parse(USART_RX_BUF,&cmd_argc,cmd_argv);  //解析命令
    if(erro_n < 0){
      //打印函数执行错误信息
      if(erro_n == -3){
        len = 0;
        //memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
        HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
        USART_RX_STA=0;
        return;
      }else if(erro_n == -2){
        uprintf("命令参数长度过长\r\n");
        //USART_SendString(CMD_USARTx,"msg: 命令参数长度过长\n");
      }else if(erro_n == -1){
        uprintf("命令参数过多\r\n");
        //USART_SendString(CMD_USARTx,"msg: 命令参数过多\n");
      }
      len = 0;
      //memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
      HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
      USART_RX_STA=0;
      return;
    }
    erro_n = cmd_exec(cmd_argc,cmd_argv);   //执行命令
    if(erro_n < 0){
      //打印函数执行错误信息
      if(erro_n == -2){
        uprintf("未找到命令%s\r\n",cmd_argv[0]);
        //USART_SendString(CMD_USARTx,"msg: 未找到命令:%s\r\n",cmd_argv[0]);
      }
      len = 0;
      //memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
      HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
      USART_RX_STA=0;
      return;
    }
    len = 0;
    //memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
    USART_RX_STA=0;
  }
}


char s[22]={'b','y',16,6};
void send_wave(float arg1,float arg2,float arg3,float arg4){
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
  s[2]=16;  //length
  s[3]=6;   //type
  s[20]='\r';
  s[21]='\n';
  memcpy(s+4,&arg1,sizeof(arg1));
  memcpy(s+8,&arg2,sizeof(arg1));
  memcpy(s+12,&arg3,sizeof(arg1));
  memcpy(s+16,&arg4,sizeof(arg1));
  HAL_UART_Transmit(&huart1,(uint8_t *)s, 22,1000);
 // HAL_UART_Transmit_DMA(&huart1,(uint8_t *)s, 22);   
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
