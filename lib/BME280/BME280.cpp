#include "mbed.h"
#include "BME280.h"
 
/* internal registers */
#define REG_HUM_LSB    0xFE
#define REG_HUM_MSB    0xFD
#define REG_TEMP_XLSB  0xFC
#define REG_TEMP_LSB   0xFB
#define REG_TEMP_MSB   0xFA
#define REG_PRESS_XLSB 0xF9
#define REG_PRESS_LSB  0xF8
#define REG_PRESS_MSB  0xF7
#define REG_CONFIG     0xF5
#define REG_CTRL_MEAS  0xF4
#define REG_STATUS     0xF3
#define REG_CTRL_HUM   0xF2
#define REG_RESET      0xE0
#define REG_ID         0xD0
 
#define REG_CALIB00   0x88
#define REG_CALIB25   0xA1
#define REG_CALIB26   0xE1
#define REG_CALIB41   0xF0
 
/** Trimming parameters */
/* temperature */
#define REG_T1_LSB 0x88
#define REG_T1_MSB 0x89
#define REG_T2_LSB 0x8A
#define REG_T2_MSB 0x8B
#define REG_T3_LSB 0x8C
#define REG_T3_MSB 0x8D
/* pressure */
#define REG_P1_LSB 0x8E
#define REG_P1_MSB 0x8F
#define REG_P2_LSB 0x90
#define REG_P2_MSB 0x91
#define REG_P3_LSB 0x92
#define REG_P3_MSB 0x93
#define REG_P4_LSB 0x94
#define REG_P4_MSB 0x95
#define REG_P5_LSB 0x96
#define REG_P5_MSB 0x97
#define REG_P6_LSB 0x98
#define REG_P6_MSB 0x99
#define REG_P7_LSB 0x9A
#define REG_P7_MSB 0x9B
#define REG_P8_LSB 0x9C
#define REG_P8_MSB 0x9D
#define REG_P9_LSB 0x9E
#define REG_P9_MSB 0x9F
/* humidity */
#define REG_H1     0xA1
#define REG_H2_LSB 0xE1
#define REG_H2_MSB 0xE2
#define REG_H3     0xE3
#define REG_H4     0xE4
#define REG_H5     0xE5
#define REG_H6     0xE7
 
/*
 * Register description
 * 0xD0 "id"
 *  chip identification number chip_id[7:0], which is 0x60
 *
 * 0xE0 "reset"
 *  Software reset register, writing 0xB6 causes complete power-on-reset.
 *
 * 0xF2 "ctrl_hum"
 *  Specifies the humidity data acquision options.
 *  Note: Changes to this register only become effective after a write
 *  operaion to "ctrl_meas".
 * bit[2:0] osrs_h[2:0] Controls oversampling of humidity data.
 *  000: Skipped (output set to 0x8000)
 *  001: oversampling x1
 *  010: oversampling x2
 *  011: oversampling x4
 *  100: oversampling x8
 *  101, others, oversampling x16
 *
 * 0xF3 "status"
 * bit[3] measuring[0] Automatically set to '1' during conversion 
 *        and back to '0' when done
 * bit[0] im_update[0] Automatically set to '1' during NVM data copy 
 *        and back to '0' when done
 *
 * 0xF4 "ctrl_meas" 
 *  Specifies the pressure and temperature data acquision options.
 *  Note: This register needs to be written after canging "ctrl_hum" to take effect.
 * bit[7:5] osrs_t[2:0] Controls oversampling of temperature data
 *  bit value is similar to osrs_h (except 000 output set to 0x80000)
 * bit[4:2] osrs_p[2:0] Controls oversampling of pressure data
 *  bit value is similar to osrs_h (except 000 output set to 0x80000)
 * bit[1:0] mode[1:0] Controls the sensor mode.
 *  00: Sleep mode
 *  01: Forced mode
 *  10: Forced mode
 *  11: Normal mode
 *
 * 0xF5 "config" 
 *  Specifies the rate, filter and interface options.
 *  Writes to "config" in normal mode may be ignored.
 *  In sleep mode writes are not ignored.
 * bit[7:5] t_sb[2:0] Controls inactive duration t_standby in normal mode.
 * 000:    0.5 [ms]
 * 001:   62.5 [ms]
 * 010:  125   [ms]
 * 011:  250   [ms]
 * 100:  500   [ms]
 * 101: 1000   [ms]
 * 110:   10   [ms]
 * 111:   20   [ms]
 * bit[4:2] filter[2:0] Controls the time constant of the IIR filter.
 * 000: Filter off
 * 001:  2
 * 010:  4
 * 011:  8
 * 100, others: 16
 *
 * 0xF7 .. 0xF9 "press" (_msb, _lsb, _xlsb)
 * 0xF7 bit[7:0] press_msb[7:0]   MSB part up[19:12]
 * 0xF8 bit[7:0] press_lsb[7:0]   LSB part  up[11:4]
 * 0xF9 bit[7:4] press_xlsb[3:0] XLSB part   up[3:0]
 *
 * 0xFA .. 0xFC "temp" (_msb, _lsb, _xlsb)
 * 0xFA bit[7:0] temp_msb[7:0]   MSB part ut[19:12]
 * 0xFB bit[7:0] temp_lsb[7:0]   LSB part  ut[11:4]
 * 0xFC bit[7:4] temp_xlsp[3:0] XLSB part   ut[3:0]
 *
 * 0xFD .. 0xFE "hum" (_msb, _lsb)
 * 0xFD bit[7:0] hum_msb[7:0]    MSB part uh[15:8]
 * 0xFE bit[7:0] hum_lsb[7:0]    LSB part  uh[7:0]
 */
 /*
  * Compensation parameter storage, naming and data type
  * Register addr,  Register content,    Data type
  * 0x88/0x89,      dig_T1[7:0]/[15:8],  unsigned short
  * 0x8A/0x8B,      dig_T2[7:0]/[15:8],    signed short
  * 0x8C/0x8D,      dig_T3[7:0]/[15:8],    signed short
  * 0x8E/0x8F,      dig_P1[7:0]/[15:8],  unsigned short
  * 0x90/0x91,      dig_P2[7:0]/[15:8],    signed short
  * 0x92/0x93,      dig_P3[7:0]/[15:8],    signed short
  * 0x94/0x95,      dig_P4[7:0]/[15:8],    signed short
  * 0x96/0x97,      dig_P5[7:0]/[15:8],    signed short
  * 0x98/0x99,      dig_P6[7:0]/[15:8],    signed short
  * 0x9A/0x9B,      dig_P7[7:0]/[15:8],    signed short
  * 0x9C/0x9D,      dig_P8[7:0]/[15:8],    signed short
  * 0x9E/0x9F,      dig_P9[7:0]/[15:8],    signed short
  * 0xA1,           dig_H1[7:0],         unsigned char
  * 0xE1/0xE2,      dig_H2[7:0]/[15:8],    signed short
  * 0xE3,           dig_H3[7:0],         unsigned char
  * 0xE4/0xE5[3:0], dig_H4[11:4]/[3:0]  signed short
  * 0xE5[7:4]/0xE6, dig_H5[3:0]/[11:4]  signed short
  * 0xE7,           dig_H6[7:0],        signed char
  */
 
void BME280::init(void)
{
    uint8_t data[18] ;
    
    data[0] = REG_CTRL_HUM ;
    data[1] = 0x03 ; /* Humidity oversampling x4 */
    writeRegs(data, 2) ;
    
    data[0] = REG_CTRL_MEAS ;
    data[1] = (0x3 << 5) /* temp oversample x4 */
            | (0x3 << 2) /* pres oversample x4 */
            | (0x00)      /* Sleep Mode */
            ;
    writeRegs(data, 2) ;
    
    data[0] = REG_CONFIG ;
    data[1] = (0x4 << 5) /* standby 500ms */
            | (0x0 << 2) /* filter off */
            | (0x0)      /* spi 4wire mode */   
            ;
    writeRegs(data, 2) ;
    
    /* read dig_T regs */
    readRegs(REG_T1_LSB, data, 6) ;
    dig_T1 = (data[1] << 8) | data[0] ;
    dig_T2 = (data[3] << 8) | data[2] ;
    dig_T3 = (data[5] << 8) | data[4] ;
// printf("dig_T1:0x%04X, dig_T2:0x%04X, dig_T3:0x%04X\n",dig_T1, dig_T2, dig_T3) ;
    
    /* read dig_P regs */
    readRegs(REG_P1_LSB, data, 18) ;
    dig_P1 = (data[ 1] << 8) | data[ 0] ;
    dig_P2 = (data[ 3] << 8) | data[ 2] ;
    dig_P3 = (data[ 5] << 8) | data[ 4] ;
    dig_P4 = (data[ 7] << 8) | data[ 6] ;
    dig_P5 = (data[ 9] << 8) | data[ 8] ;
    dig_P6 = (data[11] << 8) | data[10] ;
    dig_P7 = (data[13] << 8) | data[12] ;
    dig_P8 = (data[15] << 8) | data[14] ;
    dig_P9 = (data[17] << 8) | data[16] ;
// printf("dig_P1:0x%04X, dig_P2:0x%04X, dig_P3:0x%04X\n",dig_P1, dig_P2, dig_P3) ;
// printf("dig_P4:0x%04X, dig_P5:0x%04X, dig_P6:0x%04X\n",dig_P4, dig_P5, dig_P6) ;
// printf("dig_P7:0x%04X, dig_P8:0x%04X, dig_P9:0x%04X\n",dig_P7, dig_P8, dig_P9) ;
    
    /* read dig_H regs */
    readRegs(REG_H1, data, 1) ;
    dig_H1 = data[0] ;
    readRegs(REG_H2_LSB, data, 2) ;
    dig_H2 = (data[1] << 8) | data[0] ;
    readRegs(REG_H3, data, 1) ;
    dig_H3 = data[0] ;
    readRegs(REG_H4, data, 3) ; 
    dig_H4 = (data[0] << 4) | (data[1] & 0x0F) ;
    dig_H5 = (data[2] << 4) | (data[1] >> 4) ;  
    readRegs(REG_H6, data, 1) ;
    dig_H6 = data[0] ;
// printf("dig_H1:0x%04X, dig_H2:0x%04X, dig_H3:0x%04X\n",dig_H1, dig_H2, dig_H3) ;    
// printf("dig_H4:0x%04X, dig_H5:0x%04X, dig_H6:0x%04X\n",dig_H4, dig_H5, dig_H6) ;   
 
    trigger() ; /* dummy measure */
}
 
BME280::BME280(PinName sda, PinName scl, uint8_t addr) 
{
    m_i2c = new I2C(sda, scl);
    m_i2c->frequency(400000);
    m_addr = (addr << 1);
    m_spi = 0 ;
    m_cs = 0 ;
    init() ;
}
 
BME280::BME280(PinName sck, PinName miso, PinName mosi, PinName cs) 
{
    m_cs = new DigitalOut(cs, 1) ;
    m_spi = new SPI(mosi, miso, sck) ;
    m_spi->format(8, 3) ;
    m_i2c = 0 ;
    m_addr = 0 ;
    init() ;
}
 
BME280::~BME280() 
{
    if (m_spi) { 
        delete m_spi ;
        delete m_cs ; 
    }
    if (m_i2c) { 
        delete m_i2c ; 
        m_addr = 0 ;
    }
}
 
void BME280::i2c_readRegs(uint8_t addr, uint8_t * data, int len) {
    char t[1] = {addr};
    m_i2c->write(m_addr, t, 1, true) ;
    m_i2c->read(m_addr, (char*)data, len) ;
}
 
void BME280::i2c_writeRegs(uint8_t * data, int len) {
   m_i2c->write(m_addr, (char *)data, len) ;
}
 
void BME280::spi_readRegs(uint8_t addr, uint8_t * data, int len) {
    *m_cs = 0 ;
    m_spi->write(addr | 0x80) ;
    for (int i = 0 ; i < len ; i++ ) {    
      data[i] = m_spi->write((addr+i)|0x80) ; 
    } 
    m_spi->write(0x00) ; // to terminate read mode
    *m_cs = 1 ;
}
 
void BME280::spi_writeRegs(uint8_t * data, int len) {
   *m_cs = 0 ;
   for (int i = 0 ; i < len-1 ; i++ ) {
      m_spi->write((data[0]+i)^0x80) ; /* register address */
      m_spi->write(data[i+1]) ; /* data to write */
 
   }
   *m_cs = 1 ;
}
 
void BME280::readRegs(uint8_t addr, uint8_t *data, int len) 
{
    if (m_spi) {
        spi_readRegs(addr, data, len) ;
    } else if (m_i2c) {
        i2c_readRegs(addr, data, len) ;
    }
}
 
void BME280::writeRegs(uint8_t *data, int len) 
{
    if (m_spi) {
        spi_writeRegs(data, len) ;
    } else if (m_i2c) {
        i2c_writeRegs(data, len) ;
    }
}
 
void BME280::reset(void)
{
    uint8_t data[2] ;
    data[0] = REG_RESET ;
    data[1] = 0xB6 ;
    writeRegs(data, 2) ;
}
 
void BME280::trigger(void)
{
    uint8_t data[2] ;
    readRegs(REG_CTRL_MEAS, &data[1], 1) ;
    
    data[0] = REG_CTRL_MEAS ;
    data[1] = (data[1] & 0xFC) /* keep oversampling */
            | (0x2)      /* Forced Mode */
            ;
    writeRegs(data, 2) ;
}
 
uint8_t BME280::busy(void)
{
    uint8_t data[1] ;
    readRegs(REG_STATUS, data, 1) ;
    return( data[0] & 0x9 ) ;
}
 
uint8_t BME280::getID(void)
{
    uint8_t data[1] ;
    readRegs(REG_ID, data, 1) ;
    return(data[0]) ;
}
 
float BME280::getTemperature(void) 
{
    float tempf ;
    uint8_t data[3] ;
    BME280_S32_t var1, var2, T ;
    BME280_S32_t adc_T ;
    
    readRegs(REG_TEMP_MSB, data, 3) ;
    adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    
    var1 = ((((adc_T>>3) - ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) *
        ((BME280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    tempf = (float)T;
    return (tempf/100.0f);
}
 
float BME280::getPressure(void) 
{
    BME280_S32_t adc_P ;
    BME280_S64_t var1, var2, p ;
    float pressf;
    uint8_t data[3];
 
    readRegs(REG_PRESS_MSB, data, 3) ;
    adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
 
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (BME280_S64_t)dig_P3)>>8) + ((var1 * (BME280_S64_t)dig_P2)<<12);
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33;
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BME280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7)<<4);
 
    pressf = (float)(p >> 8) ;
    return (pressf/100.0f);
}
 
float BME280::getHumidity(void) 
{
    BME280_S32_t adc_H;
    BME280_S32_t v_x1_u32r ;
    float humf;
    uint8_t data[2];
 
    readRegs(REG_HUM_MSB, data, 2) ;
    adc_H = (data[0] << 8) | data[1];
 
    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) +
        ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
        ((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
        ((BME280_S32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    humf = (float)(v_x1_u32r >> 12) ;
 
    return (humf/1024.0f);
}