/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmp109.h
 *
 * Date : 2018/3/08
 *
 * Usage: GMP109 sensor driver header file
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file gmp109.h
 *  @brief  GMP109 Sensor Driver Header File 
 *  @author Joseph FC Tseng
 */
 
#ifndef __GMP109_H__
#define __GMP109_H__

#include "bus_support.h"

#define GMP109_7BIT_I2C_ADDR		0x76
#define GMP109_PRESSURE_SENSITIVITY (16)  //1 Pa = 16 code
#define GMP109_TEMPERATURE_SENSITIVITY (100)  //1 Celsius = 100 code
#define GMP109_P_CODE_TO_PA(pCode) (((float)(pCode)) / GMP109_PRESSURE_SENSITIVITY)
#define GMP109_T_CODE_TO_CELSIUS(tCode) (((float)(tCode)) / GMP109_TEMPERATURE_SENSITIVITY)

//Registers Address
#define GMP109_REG_TEMPH	  0x02
#define GMP109_REG_TEMPM	  0x03
#define GMP109_REG_TEMPL	  0x04
#define GMP109_REG_PRESSH	  0x05
#define GMP109_REG_PRESSM	  0x06
#define GMP109_REG_PRESSL	  0x07
#define GMP109_REG_STATUS 	  0x08
#define GMP109_REG_CTRL1 	  0x09
#define GMP109_REG_CTRL2 	  0x0A
#define GMP109_REG_CTRL3 	  0x0B
#define GMP109_REG_CTRL4 	  0x0D
#define GMP109_REG_CTRL5 	  0x0F
#define GMP109_REG_RESET	  0x11

//Soft reset
#define GMP109_SW_RST_SET_VALUE		0xB6

/* DRDY bit */
#define GMP109_DRDY__REG	   GMP109_REG_STATUS
#define GMP109_DRDY__MSK	   0x04
#define GMP109_DRDY__POS	   2
/* Mode bits */
#define GMP109_MODE__REG	   GMP109_REG_CTRL1
#define GMP109_MODE__MSK	   0x03
#define GMP109_MODE__POS	   0
/* P OSRCIC bits */
#define GMP109_P_OSRCIC__REG       GMP109_REG_CTRL1
#define GMP109_P_OSRCIC__MSK       0x1C
#define GMP109_P_OSRCIC__POS       2
/* T OSRCIC bits */
#define GMP109_T_OSRCIC__REG       GMP109_REG_CTRL1
#define GMP109_T_OSRCIC__MSK       0xE0
#define GMP109_T_OSRCIC__POS       5
/* Standby time bits */
#define GMP109_STANDBY_TIME__REG   GMP109_REG_CTRL2
#define GMP109_STANDBY_TIME__MSK   0x0F
#define GMP109_STANDBY_TIME__POS   0
/* T OSRFIR bits */
#define GMP109_T_OSRFIR__REG       GMP109_REG_CTRL3
#define GMP109_T_OSRFIR__MSK       0x38
#define GMP109_T_OSRFIR__POS       3
/* P OSRFIR bits */
#define GMP109_P_OSRFIR__REG       GMP109_REG_CTRL4
#define GMP109_P_OSRFIR__MSK       0x38
#define GMP109_P_OSRFIR__POS       3
/* P IIR bits */
#define GMP109_P_IIR__REG          GMP109_REG_CTRL4
#define GMP109_P_IIR__MSK          0x07
#define GMP109_P_IIR__POS          0
/* SPI3W bit */
#define GMP109_SPI3W__REG          GMP109_REG_CTRL5
#define GMP109_SPI3W__MSK          0x01
#define GMP109_SPI3W__POS          0

#define GMP109_GET_BITSLICE(regvar, bitname)	\
  ((regvar & bitname##__MSK) >> bitname##__POS)

#define GMP109_SET_BITSLICE(regvar, bitname, val)			\
  ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

typedef enum {
  GMP109_MODE_STANDBY = 0x00,
  GMP109_MODE_FORCE = 0x01,
  GMP109_MODE_CONTINUOUS = 0x03,
} GMP109_MODE_Type;
typedef enum {
  GMP109_SKIP_MEASUREMENT = 0x00,
  GMP109_OSRCIC_64 = 0x01,
  GMP109_OSRCIC_128 = 0x02,
  GMP109_OSRCIC_256 = 0x03,
  GMP109_OSRCIC_512 = 0x04,	
} GMP109_OSRCIC_Type;

typedef enum {
  GMP109_OSRFIR_1 = 0x00,
  GMP109_OSRFIR_2 = 0x01,
  GMP109_OSRFIR_4 = 0x02,
  GMP109_OSRFIR_8 = 0x03,
  GMP109_OSRFIR_16 = 0x04,
  GMP109_OSRFIR_32 = 0x05,
  GMP109_OSRFIR_64 = 0x06,
  GMP109_OSRFIR_128 = 0x07,	
} GMP109_OSRFIR_Type;

typedef enum {
  GMP109_IIR_OFF = 0x00,
  GMP109_IIR_2 = 0x01,
  GMP109_IIR_4 = 0x02,
  GMP109_IIR_8 = 0x03,
  GMP109_IIR_16 = 0x04,	
} GMP109_IIR_Type;

typedef enum {
  GMP109_STANDBY_TIME_0ms = 0x08,
  GMP109_STANDBY_TIME_0p2ms = 0x09,
  GMP109_STANDBY_TIME_0p5ms = 0x00,
  GMP109_STANDBY_TIME_1ms = 0x0A,
  GMP109_STANDBY_TIME_2ms = 0x0B,
  GMP109_STANDBY_TIME_4ms = 0x0C,
  GMP109_STANDBY_TIME_8ms = 0x0D,
  GMP109_STANDBY_TIME_16ms = 0x0E,
  GMP109_STANDBY_TIME_32ms = 0x0F,
  GMP109_STANDBY_TIME_63ms = 0x01,
  GMP109_STANDBY_TIME_125ms = 0x02,
  GMP109_STANDBY_TIME_250ms = 0x03,
  GMP109_STANDBY_TIME_500ms = 0x04,
  GMP109_STANDBY_TIME_1000ms = 0x05,
  GMP109_STANDBY_TIME_2000ms = 0x06,
  GMP109_STANDBY_TIME_4000ms = 0x07,
} GMP109_STANDBY_TIME_Type;

typedef enum {
  GMP109_SPI4W = 0x00,
  GMP109_SPI3W = 0x01,
} GMP109_SPI_MODE_Type;

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success, number of bytes read
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
s8 gmp109_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len);

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success, number of bytes write
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmp109_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len);

/*!
 * @brief gmp109 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_bus_init(bus_support_t* pbus);

/*!
 * @brief gmp109 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_soft_reset(void);

/*!
 * @brief gmp109 force mode measure temperature and pressure
 *
 * @param *ps32P calibrated pressure in code returned to caller
 * @param *ps32T calibrated temperature in code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_force_measure_P_T(s32* ps32P, s32* ps32T);

/*!
 * @brief gmp109 read calibrated pressure and temperature
 *        This function just read data registers, thus should 
 *        be called when GMP109 is in the continuous mode that
 *        data conversion is periodically conducted.
 *        
 * @param *ps32P calibrated pressure in code returned to caller
 * @param *ps32T calibrated temperature code returned to caller
 *
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_read_P_T(s32* ps32P, s32* ps32T);

/*!
 * @brief gmp109 set mode
 *
 * @param mode set to the MODE[1:0] bits
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_set_mode(GMP109_MODE_Type mode);

/*!
 * @brief gmp109 set standby-time for continuous mode
 *
 * @param stbyTime standby time to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_set_standby_time(GMP109_STANDBY_TIME_Type stbyTime);

/*!
 * @brief gmp109 set pressure OSR
 *        P_OSR = OSRCIC_P * OSRFIR_P
 *        See datasheet "Noise and Resolution" section for description.
 *
 * @param osrcic set to OSRCIC_P bits
 * @param osrfir set to OSRFIR_P bits
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_set_P_OSR(GMP109_OSRCIC_Type osrcic, GMP109_OSRFIR_Type osrfir);

/*!
 * @brief gmp109 set temperature OSR
 *        T_OSR = OSRCIC_T * OSRFIR_T
 *        See datasheet "Noise and Resolution" section for description.
 *
 * @param osrcic set to OSRCIC_T bits
 * @param osrfir set to OSRFIR_T bits
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_set_T_OSR(GMP109_OSRCIC_Type osrcic, GMP109_OSRFIR_Type osrfir);

/*!
 * @brief gmp109 set pressure IIR filter order
 *
 * @param iir set to IIR_P bits
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_set_P_IIR(GMP109_IIR_Type iir);

/*!
 * @brief gmp109 set SPI 3/4-wire mode
 *
 * @param spi set to SPI3W bit
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp109_set_SPI_mode(GMP109_SPI_MODE_Type spi);

#endif // __GMP109_H__
