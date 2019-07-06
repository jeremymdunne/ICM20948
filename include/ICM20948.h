#ifndef _ICM20948_H_
#define _ICM20948_H_

#include <ICM20948_Registers.h>
#include <Wire.h>
#include <Arduino.h>

// error codes
#define ICM20948_UNABLE_TO_CONNECT                          -1
#define ICM20948_WHO_AM_I_INCORRECT_RESPONSE                -2
#define ICM20948_ADDR_NOT_SET                               -3
#define ICM20948_RESET_TIMEOUT                              -4
#define ICM20498_MAGNETOMETER_WHO_AM_I_INCORRECT_RESPONSE   -5

// soft errors
#define ICM20948_NO_NEW_DATA                                10

#define ICM20948_ADDR_AD0_LOW 0x69
#define ICM20948_ADDR_AD0_HIGH 0x68

enum ICM20948_Dmp_Mode{
  DMP_ON = 0x01,
  DMP_OFF = 0x00
};

class ICM20948 {

public:

  enum DmpMode{
    DMP_ON = 0x01,
    DMP_OFF = 0x00
  };

  enum ClockSource{
    CLKSEL_AUTO = 0x01,
    CLKSEL_20_MHZ = 0x06,
    CLKSEL_STOP = 0x07
  };

  enum GyroRange{
    GYRO_250_DPS = 0x00,
    GYRO_500_DPS = 0x01,
    GYRO_1000_DPS = 0x02,
    GYRO_2000_DPS = 0x03
  };

  enum GyroLowPassFilter{
    GYRO_DLPF_380_HZ = 7,
    GYRO_DLPF_230_HZ = 0,
    GYRO_DLPF_190_HZ = 1,
    GYRO_DLPF_150_HZ = 2,
    GYRO_DLPF_73_HZ = 3,
    GYRO_DLPF_36_HZ = 4,
    GYRO_DLPF_18_HZ = 5,
    GYRO_DLPF_9_HZ = 6
  };

  enum AccelLowPassFilter{
    ACCEL_DLPF_250_HZ = 0,
    ACCEL_DLPF_111_HZ = 2,
    ACCEL_DPLF_50_HZ = 3,
    ACCEL_DLPF_24_HZ = 4,
    ACCEL_DLPF_12_HZ = 5,
    ACCEL_DLPF_6_HZ = 6,
    ACCEL_DLPF_480_HZ = 7
  };

  enum AccelRange{
    ACCEL_2_GRAVITIES = 0x00,
    ACCEL_4_GRAVITIES = 0x01,
    ACCEL_8_GRAVITIES = 0x02,
    ACCEL_16_GRAVITIES = 0x03,
  };

  enum MagnetometerMode{
    MAGNETOMETER_SINGLE_READ_MODE = 0b00001,
    MAGNETOMETER_CONTINUOUS_READ_MODE_1 = 0b00010,
    MAGNETOMETER_CONTINUOUS_READ_MODE_2 = 0b00100,
    MAGNETOMETER_CONTINUOUS_READ_MODE_3 = 0b00110,
    MAGNETOMETER_CONTINUOUS_READ_MODE_4 = 0b01000
  };

  struct RawData{
    float gyro[3] = {0,0,0}; // in deg/s
    float accel[3] = {0,0,0}; // in Gs
    float mag[3] = {0,0,0}; // in microTeslas
    float temperature = 0; // in C
    ulong long timestamp = 0; //micros  
  };

  int init(int addr = ICM20948_ADDR_AD0_LOW, AccelRange a_range = ACCEL_8_GRAVITIES, GyroRange g_range = GYRO_2000_DPS, AccelLowPassFilter a_filter = ACCEL_DLPF_111_HZ, GyroLowPassFilter g_filter = GYRO_DLPF_190_HZ, bool useMag = false, bool initWire = true);
  int setAccelRange(AccelRange range);
  int setGyroRange(GyroRange range);
  int setAccelLowPassFilter(AccelLowPassFilter filter);
  int setGyroLowPassFilter(GyroLowPassFilter filter);
  int reset();
  int update();
  int getData(RawData *data);
  int setSleepMode(bool shouldSleep);
  int setStoredGyroOffsets(float *gx, float *gy, float *gz);
  void copyRawData(RawData * source, RawData * target);

private:
  bool useMag = false;
  uint icmAddr = -1;
  int curBank = 0;
  ulong long lastUpdateMicros = 0;
  float gyroOffsets[3] = {0,0,0}; //in DPS
  float gyroScale = 1.0/131; //computed from FS_SEL, this is default
  float  accelOffsets[3] = {0,0,0}; //in m/s^2
  float accelScale = 1.0/16384; //computed from FS_SEL, this is default
  float tempOffset = -21.0;
  float tempScale = 1.0/333.87;
  float magScale = 1.0/.15;
  uint16_t magOffsets[3] = {0,0,0};

  RawData rawData;

  int readAll();

  int setClockSource(ClockSource source = CLKSEL_AUTO);
  bool checkForRawData();

  int icmWrite8(uint16_t reg, uint8_t value);
  int icmWrite16(uint16_t reg, uint);
  uint8_t icmRead8(uint16_t reg);

  int icmRead(uint16_t startReg, uint8_t count, uint8_t *values);
  int icmWrite(uint16_t startReg, uint8_t count, uint8_t *values);
  uint16_t icmRead16(uint16_t reg);
  int setBank(uint8_t bank);
  uint8_t determineUserBank(uint16_t reg);

  int setMagReadWriteSize(uint8_t bytes);
  int resetMag();
  int setSlaveAddr(uint8_t addr);
  int setSlaveWrite();
  int setSlaveRead();
  int setSlaveStartRegister(uint8_t reg);
  int setSlaveI2CSpeed();
  int enableI2CMaster();
  int setMagMode(MagnetometerMode mode);
  uint8_t readAk(uint8_t reg);
  uint8_t writeAk(uint8_t reg, uint8_t value);
  int getRawMagData();
  int initMagnetometer();

};

#endif
