#ifndef _BME280_H_
#define _BME280_H_
#include "mbed.h"
 
typedef int32_t  BME280_S32_t ;
typedef uint32_t BME280_U32_t ;
typedef long long signed int BME280_S64_t ;
 
#if 0
#define BME280_U32_t uint32_t
#define BME280_S32_t int32_t
#define BME280_S64_t int64_t
#endif 
 
/**
 * BME280 Environmental sensor
 *
 * @note Note: Interface selection is done by the value of CSB (chip select)
 * @note if CSB is pulled-up, I2C interface is active.
 * @note if CSB is pulled-down, SPI interface is active.
 * @note After CSB has been pulled down once (regardless of whether any clock cycle occurred)
 * @note the I2C interface is disabled until the next power-on-reset.
 */
 
class BME280
{
public:
 /**
  * BME280 I2C Interface
  *
  * @param sda SDA pin
  * @param scl SCL pin
  * @param addr address of the I2C peripheral
  */
BME280(PinName sda, PinName scl, uint8_t addr) ;
 
 /**
  * BME280 SPI Interface
  *
  * @param sck  SPI SCKL pin
  * @param miso SPI Master In Slave Out pin
  * @param mosi SPI Master Out Slave In pin
  * @param cs   SPI Chip Select pin
  */
BME280(PinName sck, PinName miso, PinName mosi, PinName cs) ;
 
/**
 * BME280 destructor
 */
~BME280() ;
 
/** 
 * software reset
 *
 * @param none
 * @returns none
 */
void reset(void) ; 
 
/**
 * initialization
 *
 * @param none
 * @reurns none
 */
void init(void) ;
 
/**
 * trigger for one-shot measure
 *
 * @param none
 * @returns none
 */
void trigger(void) ; 
 
/**
 * check status register for the device activity
 *
 * @param none
 * @returns uint8_t measuring (0x08) or in NVM udpate (0x01)
 */
uint8_t  busy(void) ;
 
/**
 * get device ID
 *
 * @param none
 * @returns uint8_t ID for BME280 0x60
 */
uint8_t getID(void) ;
 
/**
 * get Temperature
 *
 * @param none
 * @returns float temperature in degree Celsious
 */
float getTemperature(void) ;
 
/**
 * get Humidity
 *
 * @param none
 * @returns float humidity in %
 */
float getHumidity(void) ;
 
/**
 * get Pressure
 *
 * @param none
 * @returns float pressure in hPa
 */
float getPressure(void) ;
 
private:
  SPI *m_spi ;
  I2C *m_i2c ;
  DigitalOut *m_cs ;
  char m_addr ;
  BME280_S32_t t_fine ;
  
  uint16_t dig_T1 ;
  int16_t  dig_T2 ;
  int16_t  dig_T3 ;
  uint16_t dig_P1 ; 
  int16_t  dig_P2 ;
  int16_t  dig_P3 ; 
  int16_t  dig_P4 ;
  int16_t  dig_P5 ;
  int16_t  dig_P6 ; 
  int16_t  dig_P7 ;
  int16_t  dig_P8 ;
  int16_t  dig_P9 ;
  uint8_t  dig_H1 ;
  int16_t  dig_H2 ;
  uint8_t  dig_H3 ;
  int16_t  dig_H4 ;
  int16_t  dig_H5 ; 
  int8_t   dig_H6 ;
 
  void i2c_readRegs(uint8_t addr, uint8_t *data, int len) ;
  void i2c_writeRegs(uint8_t *data, int len) ;
  void spi_readRegs(uint8_t addr, uint8_t *data, int len) ;
  void spi_writeRegs(uint8_t *data, int len) ;
  void readRegs(uint8_t addr, uint8_t *data, int len) ;
  void writeRegs(uint8_t *data, int len) ;
} ;
#endif /* _BME280_H_ */