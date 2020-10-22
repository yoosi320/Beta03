/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * branch Test !!!!
 * 
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build/build_config.h"

#include "barometer.h"

#include "system.h"
#include "bus_i2c.h"

#include "barometer_bmp280.h"
#include "barometer_spi_bmp280.h"
#define SF_KP   1040384   // 4Time  //yoosi
#define SF_KT   524288   // 4Time   //yoosi


#ifdef BARO

// BMP280, address 0x76

typedef struct bmp280_calib_param_s {  //yoosi
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       

} bmp280_calib_param_t;

static uint8_t bmp280_chip_id = 0;
static bool bmp280InitDone = false;
STATIC_UNIT_TESTED bmp280_calib_param_t bmp280_cal;
bool i2cBusWriteRegister( uint8_t reg, uint8_t data); //yoosi
bool i2cBusReadRegisterBuffer( uint8_t reg, uint8_t *data, uint8_t length); //yoosi
// uncompensated pressure and temperature
uint8_t i2cBusReadRegister(uint8_t reg);
int32_t bmp280_up = 0;
int32_t bmp280_ut = 0;

//yoosi   
int32_t i32rawPressure = 0;
int32_t i32rawTemperature   = 0;

static void bmp280_start_ut(void);
static void bmp280_get_ut(void);
#ifndef USE_BARO_SPI_BMP280
static void bmp280_start_up(void);
static void bmp280_get_up(void);
#endif
STATIC_UNIT_TESTED void bmp280_calculate(int32_t *pressure, int32_t *temperature);

bool bmp280Detect(baroDev_t *baro)
{
    //yoosi
    uint32_t h;
    uint32_t m;
    uint32_t l;
    if (bmp280InitDone)
        return true;

    delay(20);





    i2cRead(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, BMP280_CHIP_ID_REG, 1, &bmp280_chip_id);  /* read Chip Id */
    if (bmp280_chip_id != BMP280_DEFAULT_CHIP_ID)
        return false;

    // read calibration
    //i2cRead(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (uint8_t *)&bmp280_cal); //yoosi 주석처리
    // set oversampling + power mode (forced), and start sampling
    //i2cWrite(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, BMP280_CTRL_MEAS_REG, BMP280_MODE);  //yoosi 주석처리

    //yoosi init start 
    h = i2cBusReadRegister( 0x10);
    l = i2cBusReadRegister( 0x11);
    bmp280_cal.c0 = (uint32_t)h<<4 | l>>4;
    bmp280_cal.c0 = (bmp280_cal.c0&0x0800)?(0xF000|bmp280_cal.c0):bmp280_cal.c0;
    h =  i2cBusReadRegister( 0x11);
    l =  i2cBusReadRegister( 0x12);
    bmp280_cal.c1 = (uint32_t)(h&0x0F)<<8 | l;
    bmp280_cal.c1 = (bmp280_cal.c1&0x0800)?(0xF000|bmp280_cal.c1):bmp280_cal.c1;
    h =  i2cBusReadRegister( 0x13);
    m =  i2cBusReadRegister( 0x14);
    l =  i2cBusReadRegister( 0x15);
    bmp280_cal.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    bmp280_cal.c00 = (bmp280_cal.c00&0x080000)?(0xFFF00000|bmp280_cal.c00):bmp280_cal.c00;
    h =  i2cBusReadRegister( 0x15);
    m =  i2cBusReadRegister( 0x16);
    l =  i2cBusReadRegister( 0x17);
    bmp280_cal.c10 = (int32_t)(h & 0x0F) << 16 | (int32_t)m << 8 | l;
    bmp280_cal.c10 = (bmp280_cal.c10&0x080000)?(0xFFF00000|bmp280_cal.c10):bmp280_cal.c10;
    h =  i2cBusReadRegister( 0x18);
    l =  i2cBusReadRegister( 0x19);
    bmp280_cal.c01 = (uint32_t)h<<8 | l;
    h =  i2cBusReadRegister( 0x1A);
    l =  i2cBusReadRegister( 0x1B);
    bmp280_cal.c11 = (uint32_t)h<<8 | l;
    h =  i2cBusReadRegister( 0x1C);
    l =  i2cBusReadRegister( 0x1D);
    bmp280_cal.c20 = (uint32_t)h<<8 | l;
    h =  i2cBusReadRegister( 0x1E);
    l =  i2cBusReadRegister( 0x1F);
    bmp280_cal.c21 = (uint32_t)h<<8 | l;
    h =  i2cBusReadRegister( 0x20);
    l =  i2cBusReadRegister( 0x21);
    bmp280_cal.c30 = (uint32_t)h<<8 | l;

    // set oversampling + power mode (forced), and start sampling
    //busWriteRegister(busdev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
    //i2cBusWriteRegister(0x06, 0x26); // 4measurements pr.sec //64 Times                 //yoosi320 201021
	//i2cBusWriteRegister(0x07, 0xA0); // external sensor//4 measurements// 1Time      //yoosi320 201021
    i2cBusWriteRegister(0x06, 0x64); // 64 measurements pr.sec //16Times   
	i2cBusWriteRegister(0x07, 0x11); // internal sensor// 2 measurements// 2Time      957.6ms 
    
	i2cBusWriteRegister(0x08, 0x07); //contiuous pressure and temperature measurement
	i2cBusWriteRegister(0x09, 0x04); //enab1le Pshift /

    //yoosi init end 



    bmp280InitDone = true;

    // these are dummy as temperature is measured as part of pressure
    baro->ut_delay = 0;
    baro->get_ut = bmp280_get_ut;
    baro->start_ut = bmp280_start_ut;

    // only _up part is executed, and gets both temperature and pressure
    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;

    baro->start_up = bmp280_start_up;
    baro->get_up = bmp280_get_up;

    baro->calculate = bmp280_calculate;

    return true;
}

static void bmp280_start_ut(void)
{
    // dummy
}

static void bmp280_get_ut(void)
{
    // dummy
}


static void bmp280_start_up(void)
{
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    //i2cWrite(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, BMP280_CTRL_MEAS_REG, BMP280_MODE); //yoosi
}

static void bmp280_get_up(void)
{
    uint8_t BMPdata[BMP280_DATA_FRAME_SIZE];  //yoosi

    // read data from sensor
    //i2cBusReadRegisterBuffer( BMP280_PRESSURE_MSB_REG,  BMPdata,  BMP280_DATA_FRAME_SIZE);
    i2cRead(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, BMPdata);
    i32rawPressure = (int32_t)((((uint32_t)(BMPdata[0])) << 16) | (((uint32_t)(BMPdata[1])) << 8) | ((uint32_t)BMPdata[2] ));
    i32rawPressure= (i32rawPressure&0x800000) ? (0xFF000000|i32rawPressure) : i32rawPressure;

    i32rawTemperature = (int32_t)((((uint32_t)(BMPdata[3])) << 16) | (((uint32_t)(BMPdata[4])) << 8) | ((uint32_t)BMPdata[5] ));
    i32rawTemperature= (i32rawTemperature&0x800000) ? (0xFF000000|i32rawTemperature) : i32rawTemperature;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    /*
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    bmp280_cal.t_fine = var1 + var2;
    T = (bmp280_cal.t_fine * 5 + 128) >> 8;

    return T;*/  //yoosi

    //yoosi compensate_T
    float fTCompensate;
    float fTsc;

    fTsc = adc_T / (float)SF_KT;
    fTCompensate =  bmp280_cal.c0 * 0.5 + bmp280_cal.c1 * fTsc;
    return (int32_t)fTCompensate;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    /*
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
    */
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = i32rawTemperature / (float)SF_KT;
    fPsc = i32rawPressure / (float)SF_KP;
    qua2 = bmp280_cal.c10 + fPsc * (bmp280_cal.c20 + fPsc* bmp280_cal.c30);
    qua3 = fTsc * fPsc * (bmp280_cal.c11 + fPsc * bmp280_cal.c21);

    fPCompensate = bmp280_cal.c00 + fPsc * qua2 + fTsc * bmp280_cal.c01 + qua3;
    return (int32_t)fPCompensate;

}

STATIC_UNIT_TESTED void bmp280_calculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    /*
    int32_t t;
    uint32_t p;
    t = bmp280_compensate_T(bmp280_ut);
    p = bmp280_compensate_P(bmp280_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;*/
    //yoosi
    int32_t t;
    uint32_t p;
    t =(int32_t) bmp280_compensate_T(i32rawTemperature);
    p =(int32_t) bmp280_compensate_P(i32rawPressure);

    if (pressure)
        *pressure = p ;
    if (temperature)
        *temperature = t;
}

//yoosi start
bool i2cBusWriteRegister( uint8_t reg, uint8_t data)
{
    return i2cWrite(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, reg, data);
}

bool i2cBusReadRegisterBuffer( uint8_t reg, uint8_t *data, uint8_t length)
{
    return i2cRead(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, reg, length, &data);
}


uint8_t i2cBusReadRegister(uint8_t reg){
    uint8_t data;
    i2cRead(BARO_I2C_INSTANCE, BMP280_I2C_ADDR, reg, 1, &data);  
    return data;
}
// yoosi end


#endif
