#ifndef __imitate_spi_H
#define __imitate_spi_H
#ifdef __cplusplus
 extern "C" {
#endif
   
uint16_t spi_read_encoder();
void spi_read(uint8_t* readBuffer, int len);
void spi_write(uint8_t* readBuffer, int len);
void spi_write_read(uint8_t* writeBuffer, uint8_t* readBuffer, int len);
   
   
#ifdef __cplusplus
}
#endif
#endif