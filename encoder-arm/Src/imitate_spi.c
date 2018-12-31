#include "gpio.h"
#include "imitate_spi.h"
#include "usart.h"
#include "tim.h"

/*******************���ģ��spi********************/
//CPOL=0        CLK�͵�ƽ����
//CPHA=0       ��һ��ʱ���أ���ʱ�������أ��ɼ��ź�

//Ƭѡ�ź�
#define SPI1_CS_Enable() 	HAL_GPIO_WritePin(GPIOC, SPI_CS_Pin, GPIO_PIN_RESET)//Ƭѡ�źŵ͵�ƽ��Ч
#define SPI1_CS_Disable() 	HAL_GPIO_WritePin(GPIOC, SPI_CS_Pin, GPIO_PIN_SET)
//ʱ���ź�
#define SPI1_CLK_Enable() 	HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_SET                                                                                                                 )
#define SPI1_CLK_Disable() 	HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_RESET)


 /*****SPIдһ���ֽ�*****/
void spi_write_byte(uint8_t writeBuffer)
{
  int i;
  for (i=7; i>=0; i--) 
  {
    /*
    SPI1_CLK_Enable();
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, (GPIO_PinState)(writeBuffer&(1<<i)));//�Ӹ�λ7����λ0���д���д�� 
    Delay(5);       //��ʱ***********************************8
    SPI1_CLK_Disable();    // CPHA=1����ʱ�ӵĵ�һ�������ز���
    Delay(5);  
    */
    
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, (GPIO_PinState)(writeBuffer&(1<<i)));//�Ӹ�λ7����λ0���д���д�� 
    Delay(5);
    SPI1_CLK_Enable();
    Delay(5);       //��ʱ***********************************8
    SPI1_CLK_Disable();    // CPHA=1����ʱ�ӵĵ�һ�������ز���
    Delay(5);  
  }
}


/* SPI��һ���ֽ�*/
uint8_t spi_read_byte()
{
  int i;
  uint8_t Buffer=0;
  for (i=0; i<8; i++) {
    /*
    SPI1_CLK_Enable();
    Delay(5);      //��ʱ
    SPI1_CLK_Disable();   // CPHA=1����ʱ�ӵĵ�һ�������ز���
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //�Ӹ�λ7����λ0���д��ж���
    Delay(5);
   */
    SPI1_CLK_Enable();
    Delay(5);      //��ʱ
    SPI1_CLK_Disable();   // CPHA=1��
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //�Ӹ�λ7����λ0���д��ж���
    Delay(5);
  }
  return Buffer;
}

uint16_t spi_read_encoder()
{
  int i;
  uint16_t Buffer=0;
  SPI1_CS_Enable();       //���豸ʹ����Ч��ͨ�ſ�ʼ
  Delay(10); 
  for (i=0; i<10; i++) {
    /*
    SPI1_CLK_Enable();
    Delay(5);      //��ʱ
    SPI1_CLK_Disable();   // CPHA=1����ʱ�ӵĵ�һ�������ز���
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //�Ӹ�λ7����λ0���д��ж���
    Delay(5);
   */
    SPI1_CLK_Enable();
    Delay(5);      //��ʱ
    SPI1_CLK_Disable();   // CPHA=1��
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //�Ӹ�λ7����λ0���д��ж���
    Delay(5);
  }
  Delay(10);
  SPI1_CS_Disable();       //���豸ʹ����Ч��ͨ�Ž���
  Delay(10);
  return Buffer;
}

uint8_t spi_write_read_byte(uint8_t writeBuffer)
{
  int i;
  uint8_t Buffer=0;
  for (i=7;i>=0;i--) 
  {   
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, (GPIO_PinState)(writeBuffer&(1<<i)));//�Ӹ�λ7����λ0���д���д�� 
    //Delay(5);
    SPI1_CLK_Enable();
    Delay(5);
    SPI1_CLK_Disable();//�½�������
    GPIO_PinState temp=HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    Buffer = (Buffer <<1) | temp;   //�Ӹ�λ7����λ0���д��ж���
    Delay(5);
  }
  return Buffer;
}


/*
 SPIд����
 writeBuffer��д������
 len��д���ֽڵĳ���
*/
void spi_write (uint8_t* writeBuffer, int len)
{
  int i;
  SPI1_CS_Enable();       //���豸ʹ����Ч��ͨ�ſ�ʼ
  Delay(10);        //��ʱ
  //д������
  for (i=0; i<len; i++)
    spi_write_byte(writeBuffer[i]);
  Delay(10);
  SPI1_CS_Disable();       //���豸ʹ����Ч��ͨ�Ž���
  Delay(10);
}


/*
SPI������
readBuffer����������
len�������ֽڵĳ���
*/
void spi_read(uint8_t* readBuffer, int len)
{
  int i;
  SPI1_CS_Enable();       //���豸ʹ����Ч��ͨ�ſ�ʼ
  Delay(10);       //��ʱ
  //��������
  for (i=0; i<len; i++)
    readBuffer[i] = spi_read_byte();
  Delay(10);
  SPI1_CS_Disable();       //���豸ʹ����Ч��ͨ�Ž���
  Delay(10);
}

void spi_write_read(uint8_t* writeBuffer, uint8_t* readBuffer, int len)
{
  int i;
  SPI1_CS_Enable();       //���豸ʹ����Ч��ͨ�ſ�ʼ
  Delay(10); 
  for (i=0; i<len; i++)
    readBuffer[i]=spi_write_read_byte(writeBuffer[i]);
  Delay(10);
  SPI1_CS_Disable();
  Delay(10);
}