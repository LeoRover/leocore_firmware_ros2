/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  Library may be used freely and without limit with attribution.
*/

#include "ICM42605.hpp"

ICM42605::ICM42605(I2C_HandleTypeDef* i2c_bus) : _i2c_bus(i2c_bus) {}

uint8_t ICM42605::getChipID() { return readByte(ICM42605_WHO_AM_I); }

float ICM42605::getAres(uint8_t Ascale) {
  switch (Ascale) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
    default:
      return 0.0;
  }
}

float ICM42605::getGres(uint8_t Gscale) {
  switch (Gscale) {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      return _gRes;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      return _gRes;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      return _gRes;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
    default:
      return 0.0;
  }
}

void ICM42605::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR,
                    uint8_t GODR) {
  // enable gyro and accel in low noise mode
  // make sure not to disturb reserved bit values
  uint8_t temp = readByte(ICM42605_PWR_MGMT0);
  writeByte(ICM42605_PWR_MGMT0, temp | 0x0F);

  // gyro full scale and data rate
  temp = readByte(ICM42605_GYRO_CONFIG0);
  writeByte(ICM42605_GYRO_CONFIG0, temp | GODR | Gscale << 5);

  // set accel full scale and data rate
  temp = readByte(ICM42605_ACCEL_CONFIG0);
  writeByte(ICM42605_ACCEL_CONFIG0, temp | AODR | Ascale << 5);

  // set temperature sensor low pass filter to 5Hz, use first order gyro filter
  temp = readByte(ICM42605_GYRO_CONFIG1);
  writeByte(ICM42605_GYRO_CONFIG1, temp | 0xD0);
}

void ICM42605::readData(int16_t* destination) {
  uint8_t rawData[14];  // x/y/z accel register data stored here

  // Read the 14 raw data registers into data array
  readBytes(ICM42605_TEMP_DATA1, 14, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}

uint8_t ICM42605::readByte(uint16_t address) {
  uint8_t temp;
  HAL_I2C_Mem_Read(_i2c_bus, ICM42605_ADDRESS, address, 1, &temp, 1, 5);
  return temp;
}

void ICM42605::writeByte(uint16_t address, uint8_t data) {
  HAL_I2C_Mem_Write(_i2c_bus, ICM42605_ADDRESS, address, 1, &data, 1, 5);
}

void ICM42605::readBytes(uint16_t addres, uint16_t size, uint8_t* data) {
  HAL_I2C_Mem_Read(_i2c_bus, ICM42605_ADDRESS, addres, 1, data, size, 50);
}
