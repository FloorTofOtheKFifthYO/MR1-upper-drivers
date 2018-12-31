#include "gpio.h"
#include "imitate_spi.h"
#include "usart.h"
#include "tim.h"

/*******************软件模拟spi********************/
//CPOL=0        CLK低电平空闲
//CPHA=0       第一个时钟沿（即时钟上升沿）采集信号

//片选信号
#define SPI1_CS_Enable() 	HAL_GPIO_WritePin(GPIOC, SPI_CS_Pin, GPIO_PIN_RESET)//片选信号低电平有效
#define SPI1_CS_Disable() 	HAL_GPIO_WritePin(GPIOC, SPI_CS_Pin, GPIO_PIN_SET)
//时钟信号
#define SPI1_CLK_Enable() 	HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_SET                                                                                                                 )
#define SPI1_CLK_Disable() 	HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_RESET)


 /*****SPI写一个字节*****/
void spi_write_byte(uint8_t writeBuffer)
{
  int i;
  for (i=7; i>=0; i--) 
  {
    /*
    SPI1_CLK_Enable();
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, (GPIO_PinState)(writeBuffer&(1<<i)));//从高位7到低位0进行串行写入 
    Delay(5);       //延时***********************************8
    SPI1_CLK_Disable();    // CPHA=1，在时钟的第一个跳变沿采样
    Delay(5);  
    */
    
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, (GPIO_PinState)(writeBuffer&(1<<i)));//从高位7到低位0进行串行写入 
    Delay(5);
    SPI1_CLK_Enable();
    Delay(5);       //延时***********************************8
    SPI1_CLK_Disable();    // CPHA=1，在时钟的第一个跳变沿采样
    Delay(5);  
  }
}


/* SPI读一个字节*/
uint8_t spi_read_byte()
{
  int i;
  uint8_t Buffer=0;
  for (i=0; i<8; i++) {
    /*
    SPI1_CLK_Enable();
    Delay(5);      //延时
    SPI1_CLK_Disable();   // CPHA=1，在时钟的第一个跳变沿采样
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //从高位7到低位0进行串行读出
    Delay(5);
   */
    SPI1_CLK_Enable();
    Delay(5);      //延时
    SPI1_CLK_Disable();   // CPHA=1，
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //从高位7到低位0进行串行读出
    Delay(5);
  }
  return Buffer;
}

uint16_t spi_read_encoder()
{
  int i;
  uint16_t Buffer=0;
  SPI1_CS_Enable();       //从设备使能有效，通信开始
  Delay(10); 
  for (i=0; i<10; i++) {
    /*
    SPI1_CLK_Enable();
    Delay(5);      //延时
    SPI1_CLK_Disable();   // CPHA=1，在时钟的第一个跳变沿采样
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //从高位7到低位0进行串行读出
    Delay(5);
   */
    SPI1_CLK_Enable();
    Delay(5);      //延时
    SPI1_CLK_Disable();   // CPHA=1，
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //从高位7到低位0进行串行读出
    Delay(5);
  }
  Delay(10);
  SPI1_CS_Disable();       //从设备使能无效，通信结束
  Delay(10);
  return Buffer;
}

uint8_t spi_write_read_byte(uint8_t writeBuffer)
{
  int i;
  uint8_t Buffer=0;
  for (i=7;i>=0;i--) 
  {   
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, (GPIO_PinState)(writeBuffer&(1<<i)));//从高位7到低位0进行串行写入 
    //Delay(5);
    SPI1_CLK_Enable();
    Delay(5);
    SPI1_CLK_Disable();//下降沿输样
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //从高位7到低位0进行串行读出
    Delay(5);
  }
  return Buffer;
}


/*
 SPI写操作
 writeBuffer：写缓冲区
 len：写入字节的长度
*/
void spi_write (uint8_t* writeBuffer, int len)
{
  int i;
  SPI1_CS_Enable();       //从设备使能有效，通信开始
  Delay(10);        //延时
  //写入数据
  for (i=0; i<len; i++)
    spi_write_byte(writeBuffer[i]);
  Delay(10);
  SPI1_CS_Disable();       //从设备使能无效，通信结束
  Delay(10);
}


/*
SPI读操作
readBuffer：读缓冲区
len：读入字节的长度
*/
void spi_read(uint8_t* readBuffer, int len)
{
  int i;
  SPI1_CS_Enable();       //从设备使能有效，通信开始
  Delay(10);       //延时
  //读入数据
  for (i=0; i<len; i++)
    readBuffer[i] = spi_read_byte();
  Delay(10);
  SPI1_CS_Disable();       //从设备使能无效，通信结束
  Delay(10);
}

void spi_write_read(uint8_t* writeBuffer, uint8_t* readBuffer, int len)
{
  int i;
  SPI1_CS_Enable();       //从设备使能有效，通信开始
  Delay(10); 
  for (i=0; i<len; i++)
    readBuffer[i]=spi_write_read_byte(writeBuffer[i]);
  Delay(10);
  SPI1_CS_Disable();
  Delay(10);
}