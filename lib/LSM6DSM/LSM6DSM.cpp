/******************************************************************************
SparkFunLSM6DS3.cpp
LSM6DS3 Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

//See SparkFunLSM6DS3.h for additional topology notes.

#include "LSM6DSM.h"
#include "stdint.h"
#include <Arduino.h>

#include "SPI.h"

LSM6DS3Core::LSM6DS3Core( uint8_t cs,  uint8_t cs2){
		chipSelectPin = cs;
}

status_t LSM6DS3Core::beginCore(void)
{
	status_t returnError = IMU_SUCCESS;

		// start the SPI library:
		SPI.begin();
		// Maximum SPI frequency is 10MHz, could divide by 2 here:
		//SPI.setClockDivider(SPI_CLOCK_DIV4);
		// Data is read and written MSb first.
		//SPI.setBitOrder(MSBFIRST);
		// Data is captured on rising edge of clock (CPHA = 0)
		// Base value of the clock is HIGH (CPOL = 1)

		//SPI.setDataMode(SPI_MODE0);

		
		// initalize the  data ready and chip select pins:
		pinMode(chipSelectPin, OUTPUT);
		digitalWrite(chipSelectPin, HIGH);

	//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ )
	{
		temp++;
	}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
	if( readCheck != 0x6A )
	{
		returnError = IMU_HW_ERROR;
	}

	return returnError;

}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LSM6DS3Core::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;
	uint8_t tempFFCounter = 0;

SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		while ( i < length ) // slave may send less than requested
		{
			c = SPI.transfer(0x00); // receive a byte as character
			if( c == 0xFF )
			{
				//May have problem
				tempFFCounter++;
			}
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		if( tempFFCounter == i )
		{
			//Ok, we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
SPI.endTransaction();
	return returnError;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS3Core::readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	status_t returnError = IMU_SUCCESS;
SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		result = SPI.transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
		SPI.endTransaction();
		if( result == 0xFF )
		{
			//we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}

	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS3Core::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	uint8_t myBuffer[2];
	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	*outputPointer = output;
	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t LSM6DS3Core::writeRegister(uint8_t offset, uint8_t dataToWrite) {
	status_t returnError = IMU_SUCCESS;
		SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset);
		// send a value of 0 to read the first byte returned:
		SPI.transfer(dataToWrite);
		// decrement the number of bytes left to read:
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
		SPI.endTransaction();

	return returnError;
}

status_t LSM6DS3Core::embeddedPage( void )
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80 );
	
	return returnError;
}

status_t LSM6DS3Core::basePage( void )
{
	status_t returnError = writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00 );
	
	return returnError;
}


//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LSM6DS3::LSM6DS3(  uint8_t busType, uint8_t inputArg ) : LSM6DS3Core( busType, inputArg  )
{
	//Construct with these default settings

	settings.gyroEnabled = 1;  //Can be 0 or 1
	settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
	settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
	settings.gyroFifoDecimation = 1;  //set 1 for on /1

	settings.accelEnabled = 1;
	settings.accelODROff = 1;
	settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
	settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
	settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
	settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
	settings.accelFifoDecimation = 1;  //set 1 for on /1

	settings.tempEnabled = 1;

	//Select interface mode
	settings.commMode = 1;  //Can be modes 1, 2 or 3

	//FIFO control data
	settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
	settings.fifoSampleRate = 10;  //default 10Hz
	settings.fifoModeWord = 0;  //Default off

	allOnesCounter = 0;
	nonSuccessCounter = 0;

    _ares = settings.accelRange / 32768.f;
    _gres = settings.gyroRange / 32768.f;

}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t LSM6DS3::configure()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	//Begin the inherited core.  This gets the physical wires connected
	status_t returnError = IMU_SUCCESS; // = beginCore();

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	if ( settings.accelEnabled == 1) {
		//Build config reg
		//First patch in filter bandwidth
		switch (settings.accelBandWidth) {
		case 50:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
			break;
		case 100:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
			break;
		case 200:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
			break;
		default:  //set default case to max passthrough
		case 400:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
			break;
		}
		//Next, patch in full scale
		switch (settings.accelRange) {
		case 2:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
			break;
		case 4:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
			break;
		case 8:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
			break;
		default:  //set default case to 16(max)
		case 16:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
			break;
		}
		//Lastly, patch in accelerometer ODR
		switch (settings.accelSampleRate) {
		case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
			break;
		case 13330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}

	//Now, write the patched together data
	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
	if ( settings.accelODROff == 1) {
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}
	writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);

	//Setup the gyroscope**********************************************
	dataToWrite = 0; //Start Fresh!
	if ( settings.gyroEnabled == 1) {
		//Build config reg
		//First, patch in full scale
		switch (settings.gyroRange) {
		case 125:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
			break;
		case 245:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
			break;
		case 500:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
			break;
		case 1000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
			break;
		default:  //Default to full 2000DPS range
		case 2000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
			break;
		}
		//Lastly, patch in gyro ODR
		switch (settings.gyroSampleRate) {
		case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}
	//Write the byte
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

	//Setup the internal temperature sensor
	if ( settings.tempEnabled == 1) {}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
	if( readCheck != 0x6A )
	{
		returnError = IMU_HW_ERROR;
	}
	return returnError;
}


 status_t LSM6DS3::read_all(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
	uint8_t rawData[14];
	status_t returnError = readRegisterRegion(&rawData[2], LSM6DS3_ACC_GYRO_OUTX_L_G, 12);  //Does memory transfer
	// skip temp
    *gx = calcGyro(((int16_t)rawData[3] << 8) | rawData[2]) - settings._gyroBias[0];  
    *gy = -calcGyro(((int16_t)rawData[5] << 8) | rawData[4]) - settings._gyroBias[1];  
    *gz = -calcGyro(((int16_t)rawData[7] << 8) | rawData[6]) - settings._gyroBias[2];   
    *ax = calcAccel(((int16_t)rawData[9] << 8) | rawData[8]) - settings._accelBias[0];  
    *ay = -calcAccel(((int16_t)rawData[11] << 8) | rawData[10]) - settings._accelBias[1];  
    *az = -calcAccel(((int16_t)rawData[13] << 8) | rawData[12]) - settings._accelBias[2];  
	return returnError;
}


 status_t LSM6DS3::read_all_temp(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp){
	uint8_t rawData[14];
	status_t returnError = readRegisterRegion(&rawData[0], LSM6DS3_ACC_GYRO_OUT_TEMP_L, 14);  //Does memory transfer
	*temp = (float)(((int16_t)rawData[1] << 8) | rawData[0])/256 +25 ;  // Turn the MSB and LSB into a signed 16-bit value
    *gx = calcGyro(((int16_t)rawData[3] << 8) | rawData[2]) - settings._gyroBias[0];  
    *gy = -calcGyro(((int16_t)rawData[5] << 8) | rawData[4]) - settings._gyroBias[1];  
    *gz = -calcGyro(((int16_t)rawData[7] << 8) | rawData[6]) - settings._gyroBias[2];   
    *ax = calcAccel(((int16_t)rawData[9] << 8) | rawData[8]) - settings._accelBias[0];  
    *ay = -calcAccel(((int16_t)rawData[11] << 8) | rawData[10]) - settings._accelBias[1];  
    *az = -calcAccel(((int16_t)rawData[13] << 8) | rawData[12]) - settings._accelBias[2];  
	return returnError;
}
//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LSM6DS3::readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_XL );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS3::readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t LSM6DS3::readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_XL );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS3::readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t LSM6DS3::readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_XL );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS3::readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float LSM6DS3::calcAccel( int16_t input )
{
	float output = (float)input * 0.061 * (settings.accelRange >> 1) / 100;
	return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t LSM6DS3::readRawGyroX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS3::readFloatGyroX( void )
{
	float output = calcGyro(readRawGyroX());
	return output;
}

int16_t LSM6DS3::readRawGyroY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS3::readFloatGyroY( void )
{
	float output = calcGyro(readRawGyroY());
	return output;
}

int16_t LSM6DS3::readRawGyroZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_G );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LSM6DS3::readFloatGyroZ( void )
{
	float output = calcGyro(readRawGyroZ());
	return output;
}

float LSM6DS3::calcGyro( int16_t input )
{
	uint8_t gyroRangeDivisor = settings.gyroRange / 125;
	if ( settings.gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	float output = (float)input * 4.375 * (gyroRangeDivisor) / 100000;
	return output;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LSM6DS3::readRawTemp( void )
{
	int16_t output;
	readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUT_TEMP_L );
	return output;
}  

float LSM6DS3::readTempC( void )
{
	float output = (float)readRawTemp() / 256; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;

}

float LSM6DS3::readTempF( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;

}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LSM6DS3::fifoBegin( void ) {
	//CONFIGURE THE VARIOUS FIFO SETTINGS
	//
	//
	//This section first builds a bunch of config words, then goes through
	//and writes them all.

	//Split and mask the threshold
	uint8_t thresholdLByte = settings.fifoThreshold & 0x00FF;
	uint8_t thresholdHByte = (settings.fifoThreshold & 0x0F00) >> 8;
	//Pedo bits not configured (ctl2)

	//CONFIGURE FIFO_CTRL3
	uint8_t tempFIFO_CTRL3 = 0;
	if (settings.gyroFifoEnabled == 1)
	{
		//Set up gyro stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (settings.gyroFifoDecimation & 0x07) << 3;

	}
	if (settings.accelFifoEnabled == 1)
	{
		//Set up accelerometer stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (settings.accelFifoDecimation & 0x07);
	}

	//CONFIGURE FIFO_CTRL4  (nothing for now-- sets data sets 3 and 4
	uint8_t tempFIFO_CTRL4 = 0;


	//CONFIGURE FIFO_CTRL5
	uint8_t tempFIFO_CTRL5 = 0;
	switch (settings.fifoSampleRate) {
	default:  //set default case to 10Hz(slowest)
	case 10:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz;
		break;
	case 25:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz;
		break;
	case 50:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz;
		break;
	case 100:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
		break;
	case 200:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
		break;
	case 400:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz;
		break;
	case 800:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz;
		break;
	case 1600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz;
		break;
	case 3300:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz;
		break;
	case 6600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz;
		break;
	}
	//Hard code the fifo mode here:
	tempFIFO_CTRL5 |= settings.fifoModeWord = 6;  //set mode:

	//Write the data
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
	//Serial.println(thresholdLByte, HEX);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);
	//Serial.println(thresholdHByte, HEX);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

}
void LSM6DS3::fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x1000 ) == 0 ) {
		fifoRead();
	}

}
int16_t LSM6DS3::fifoRead( void ) {
	//Pull the last data from the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);
	tempAccumulator |= ((uint16_t)tempReadByte << 8);

	return tempAccumulator;
}

uint16_t LSM6DS3::fifoGetStatus( void ) {
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS1);
	tempAccumulator = tempReadByte;
	readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS2);
	tempAccumulator |= (tempReadByte << 8);

	return tempAccumulator;  

}
void LSM6DS3::fifoEnd( void ) {
	// turn off the fifo
	writeRegister(LSM6DS3_ACC_GYRO_FIFO_STATUS1, 0x00);  //Disable
}

void LSM6DS3::setCalibration(float * gyroBias, float * accelBias)
{
	for (uint8_t k=0; k<3; ++k) {
        settings._accelBias[k] = accelBias[k];
        settings._gyroBias[k] = gyroBias[k];
    }
}

void LSM6DS3::calibrate(float * gyroBias, float * accelBias)
{
    float temp[7] = {0, 0, 0, 0, 0, 0, 0};
    float sum[7] = {0, 0, 0, 0, 0, 0, 0};

    uint8_t rawData[14];
 

    for (int ii = 0; ii < 128; ii++){
	    status_t returnError = readRegisterRegion(&rawData[0], LSM6DS3_ACC_GYRO_OUT_TEMP_L, 14);  //Does memory transfer
		temp[0] = (float)(((int16_t)rawData[1] << 8) | rawData[0])/256 +25 ;  // Turn the MSB and LSB into a signed 16-bit value
	    temp[1] = calcGyro(((int16_t)rawData[3] << 8) | rawData[2]);  
	    temp[2] = -calcGyro(((int16_t)rawData[5] << 8) | rawData[4]);  
	    temp[3] = -calcGyro(((int16_t)rawData[7] << 8) | rawData[6]);   
	    temp[4] = calcAccel(((int16_t)rawData[9] << 8) | rawData[8]);  
	    temp[5] = -calcAccel(((int16_t)rawData[11] << 8) | rawData[10]);  
	    temp[6] = -calcAccel(((int16_t)rawData[13] << 8) | rawData[12]); 
       
        sum[1] += temp[1]/128.0f;
        sum[2] += temp[2]/128.0f;
        sum[3] += temp[3]/128.0f;
        sum[4] += temp[4]/128.0f;
        sum[5] += temp[5]/128.0f;
        sum[6] += temp[6]/128.0f;
        delay(50);
    }

    gyroBias[0] = sum[1];
    gyroBias[1] = sum[2];
    gyroBias[2] = sum[3];
    accelBias[0] = sum[4];
    accelBias[1] = sum[5];
    accelBias[2] = sum[6];

    if(accelBias[0] > 8.0f)  {accelBias[0] -= 10.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(accelBias[0] < -8.0f  ) {accelBias[0] += 10.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(accelBias[1] > 8.0f  )  {accelBias[1] -= 10.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(accelBias[1] < -8.0f  ) {accelBias[1] += 10.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(accelBias[2] > 8.0f  )  {accelBias[2] -= 10.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
    if(accelBias[2] < -8.0f  ) {accelBias[2] += 10.0f;}  // Remove gravity from the z-axis accelerometer bias calculation

    for (uint8_t k=0; k<3; ++k) {
        settings._accelBias[k] = accelBias[k];
        settings._gyroBias[k] = gyroBias[k];
    }

}

int LSM6DS3::write_check_reg(uint8_t adress, uint8_t data){

	writeRegister(adress, data);
	delay(2);
	uint8_t output;
	readRegister(&output, adress);
	if(output == data){
		return 0;
	}
	return 1;
}

bool LSM6DS3::enable_motion_detection( uint8_t level ){
// 1. Write 80h to FUNC_CFG_ACCESS // Enable access to embedded functions registers (bank A)
// 2. Write 08h to SM_THS // Set significant motion threshold
// 3. Write 00h to FUNC_CFG_ACCESS // Disable access to embedded functions registers (bank A)
// 4. Write 20h to CTRL1_XL// Turn on the accelerometer // ODR_XL = 26 Hz, FS_XL = ±2 g
// 5. Write 05h to CTRL10_C// Enable embedded functions // Enable significant motion detection
// 6. Write 40h to INT1_CTRL // Significant motion interrupt driven to INT1 pin

	
	// int res=0;
	// res += write_check_reg(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80);
	// res += write_check_reg(LSM6DS3_ACC_GYRO_SM_STEP_THS, 0x01);
	// res += write_check_reg(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00);
	// res += write_check_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x28); // acc 26 Hz +-4g
	// res += write_check_reg(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); // gyro off
	// res += write_check_reg(LSM6DS3_ACC_GYRO_CTRL10_C, 0x05); //
	// res += write_check_reg(LSM6DS3_ACC_GYRO_ORIENT_CFG_G, 0x80); //
	// res += write_check_reg(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x40); //
	//res += write_check_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0xE0); // Enable interrupt output and inactivity auton power saving
	//res += write_check_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0xE0);


// https://www.st.com/resource/en/datasheet/lsm6dsm.pdf
//https://www.st.com/content/ccc/resource/technical/document/application_note/group0/9a/61/96/60/49/2c/44/bc/DM00352102/files/DM00352102.pdf/jcr:content/translations/en.DM00352102.pdf

	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60); // 416 Hz ACC Mode, +- 2g, BW @ 1.5khz
	writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0xFE); // enable interrupt, ACC ODR 12.5 Hz, gyro power down, HPF on, X, Y, Z on, interrupt not latched
	writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00); // no duration
	writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x01); // 32mg
	writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20); // wake up event on INT1 enabled
	//delay(4);
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x10); // 
	//writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00); // 
	
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); // gyro off
	
	

	return 0;

	}
bool LSM6DS3::enable_normal_mode(){
	writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x01); // Reset
	delayMicroseconds(55);
	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x88); // 1,66khz Hz +-4g
	writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x00); // disable interrupts
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0xA2); // gyro 1,66 khz 250 dps, 
	writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00); // gyro enable high performance mode
	return true;

	}
bool LSM6DS3::enable_sleep(){
	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00); // power down ACC
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); // gyro off
	return true;
	}

// test int1 to check if conenction is working
bool LSM6DS3::enable_dri(){
	//writeRegister(LSM6DS3_DRDY_PULSE_CFG, 0x00); // latch mode int pin config
	
	writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x01); // acc data ready int enbled

	return true;
	}

bool LSM6DS3::disable_dri(){
	writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x00); // acc data ready int enbled

	return true;
	}

bool LSM6DS3::enable_fifo(){
	//writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x01); // acc data ready int enbled
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); // gyro off
	return true;
	}

void LSM6DS3::reset(){
	writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x01); // SW_RESET
}