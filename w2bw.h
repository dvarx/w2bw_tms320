/*
 * w2bw.h
 *
 *  Created on: Jun 10, 2022
 *      Author: dvarx
 */

#ifndef W2BW_H_
#define W2BW_H_

#define HEARTBEAT_PIN 0
#define POWER_SUPPLY_PIN 1
#define SDA_PIN 104
#define SCL_PIN 105

//sensor definitions
//---------------------------------
//the A0 version of the sensor is on the breakout board from Infineon
//the A1 version of the sensor is on the thin breakoutboard for the endoscope
#define SENSOR_A0
//#define SENSOR_A1


//range definitions
//---------------------------------
//#define SHORT_RANGE         //-50mT to 50mT , 30.8bits per mT
#define LONG_RANGE          // -150mT to 150mT , 7.7bits per mT



#ifdef SENSOR_A0
    #define I2C_W2BW_ADDRESS 0x35
#endif

#ifdef SENSOR_A1
    #define I2C_W2BW_ADDRESS 0x22
#endif



#endif /* W2BW_H_ */
