/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmp109.c
 *
 * Date : 2018/3/08
 *
 * Usage: GMP109 sensor driver file
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
 
/*! @file gmp109.c
 *  @brief  GMP109 Sensor Driver File 
 *  @author Joseph FC Tseng
 */
 
#include <stddef.h>
#include "nrf_error.h"
#include "gmp109.h"
 
#define WAIT_FOR_DRDY_LOOP_DELAY(count) {int i;for(i = 0; i < (count); ++i);}
 
bus_support_t* pGMP109Bus = 0;

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
s8 gmp109_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pGMP109Bus == NULL){
    return -127;
  }
  else{
    comRslt = pGMP109Bus->bus_read(pGMP109Bus->p_app_twi, pGMP109Bus->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes read
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;
}
 
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
s8 gmp109_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1, i;
  u8 u8Buf[2*u8Len];
  
  if(pGMP109Bus == NULL){
    return -127;
  }
  else{
    //
    // GMP109 multi-write does not support auto address increasement
    // Need to specify address for every write
    //
    for(i = 0; i < u8Len; ++i){
      u8Buf[2*i] = u8Addr + i;
      u8Buf[2*i + 1] = pu8Data[i];
    }
    comRslt = pGMP109Bus->bus_write(pGMP109Bus->p_app_twi, pGMP109Bus->u8DevAddr, u8Buf[0], &u8Buf[1], 2*u8Len - 1);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes write
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;	
}

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
s8 gmp109_bus_init(bus_support_t* pbus){
	
  u8 u8Data;
	
  //assign the I2C/SPI bus
  if(pbus == NULL)
    return -127;
  else
    pGMP109Bus = pbus;
	
  return 0;
}
 
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
s8 gmp109_soft_reset(void){
	
  s8 comRslt = -1;
  u8 u8Data = GMP109_SW_RST_SET_VALUE;
	
  //Set 11h = 0xB6
  comRslt = gmp109_burst_write(GMP109_REG_RESET, &u8Data, 1);
	
  return comRslt;
}

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
s8 gmp109_set_mode(GMP109_MODE_Type mode){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 09h
  s8Tmp = gmp109_burst_read(GMP109_MODE__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;

  // Set 09h[1:0] = mode
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_MODE, mode);
  s8Tmp = gmp109_burst_write(GMP109_MODE__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
  
}

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
s8 gmp109_set_standby_time(GMP109_STANDBY_TIME_Type stbyTime){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 0Ah
  s8Tmp = gmp109_burst_read(GMP109_STANDBY_TIME__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 0Ah[3:0] Standby_Time bits
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_STANDBY_TIME, stbyTime);
  s8Tmp = gmp109_burst_write(GMP109_STANDBY_TIME__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
  
}

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
s8 gmp109_force_measure_P_T(s32* ps32P, s32* ps32T){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data[6];

  //Set force mode
  s8Tmp = gmp109_set_mode(GMP109_MODE_FORCE);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Wait for 08h[2] DRDY bit set
  do{

    //wait a while
    WAIT_FOR_DRDY_LOOP_DELAY(1000)
		
      s8Tmp = gmp109_burst_read(GMP109_REG_STATUS, u8Data, 1);

    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    comRslt += s8Tmp;		
		
  } while( GMP109_GET_BITSLICE(u8Data[0], GMP109_DRDY) != 1);
	
  // Read 02h~07h
  s8Tmp = gmp109_burst_read(GMP109_REG_TEMPH, u8Data, 6);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  s8Tmp = sizeof(*ps32P)*8 - 24;
  // Get the calibrated temperature in code
  *ps32T = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
  *ps32T = (*ps32T << s8Tmp) >> s8Tmp; //24 bit sign extension
  // Get the calibrated pressure in code
  *ps32P = (u8Data[3] << 16) + (u8Data[4] << 8) + u8Data[5];
  *ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension	
	
 EXIT:
  return comRslt;
}

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
s8 gmp109_read_P_T(s32* ps32P, s32* ps32T){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data[6];
		
  // Read data registers 02h~07h
  s8Tmp = gmp109_burst_read(GMP109_REG_TEMPH, u8Data, 6);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  s8Tmp = sizeof(*ps32P)*8 - 24;
  // Get the calibrated temperature in code
  *ps32T = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
  *ps32T = (*ps32T << s8Tmp) >> s8Tmp; //24 bit sign extension
  // Get the calibrated pressure in code
  *ps32P = (u8Data[3] << 16) + (u8Data[4] << 8) + u8Data[5];
  *ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension	
	
 EXIT:
  return comRslt;
}

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
s8 gmp109_set_P_OSR(GMP109_OSRCIC_Type osrcic, GMP109_OSRFIR_Type osrfir){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 09h
  s8Tmp = gmp109_burst_read(GMP109_P_OSRCIC__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 09h[4:2] OSRCIC_P bits
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_P_OSRCIC, osrcic);
  s8Tmp = gmp109_burst_write(GMP109_P_OSRCIC__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;

  //Read 0Dh
  s8Tmp = gmp109_burst_read(GMP109_P_OSRFIR__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 0Dh[5:3] OSRFIR_P bits
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_P_OSRFIR, osrfir);
  s8Tmp = gmp109_burst_write(GMP109_P_OSRFIR__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}

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
s8 gmp109_set_T_OSR(GMP109_OSRCIC_Type osrcic, GMP109_OSRFIR_Type osrfir){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 09h
  s8Tmp = gmp109_burst_read(GMP109_T_OSRCIC__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 09h[7:5] OSRCIC_T bits
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_T_OSRCIC, osrcic);
  s8Tmp = gmp109_burst_write(GMP109_T_OSRCIC__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;

  //Read 0Bh
  s8Tmp = gmp109_burst_read(GMP109_T_OSRFIR__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 0Bh[5:3] OSRFIR_T bits
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_T_OSRFIR, osrfir);
  s8Tmp = gmp109_burst_write(GMP109_T_OSRFIR__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}

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
s8 gmp109_set_P_IIR(GMP109_IIR_Type iir){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 0Dh
  s8Tmp = gmp109_burst_read(GMP109_P_IIR__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 0Dh[2:0] IIR_P bits
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_P_IIR, iir);
  s8Tmp = gmp109_burst_write(GMP109_P_IIR__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}

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
s8 gmp109_set_SPI_mode(GMP109_SPI_MODE_Type spi){

  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Read 0Fh
  s8Tmp = gmp109_burst_read(GMP109_SPI3W__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the 0Fh[0] SPI3W bit
  u8Data = GMP109_SET_BITSLICE(u8Data, GMP109_SPI3W, spi);
  s8Tmp = gmp109_burst_write(GMP109_SPI3W__REG, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}
