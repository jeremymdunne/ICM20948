#include <ICM20948.h>

int ICM20948::getData(RawData *data){
  //call update to get most recent data
  update();
  //copy the information
  copyRawData(&rawData, data);
  return 0;
}

void ICM20948::copyRawData(RawData * source, RawData * target){
  for(int i = 0; i < 3; i ++){
    target->gyro[i] = source->gyro[i];
    target->accel[i] = source->accel[i];
    if(useMag) target->mag[i] = source->mag[i];
  }
  target->temperature = source->temperature;
  target->timestamp = source->timestamp;
  Serial.println(useMag);
}

int ICM20948::update(){
  //go get new data
  int status = readAll();
  return status;
}

int ICM20948::readAll(){
  if(!checkForRawData()) return ICM20948_NO_NEW_DATA;
  uint8_t tempData[20];
  icmRead(ICM20948_ACCEL_XOUT_H_REG, 20, &tempData[0]);
  rawData.accel[0] = (int16_t)(tempData[0]<<8 | tempData[1]) * accelScale - accelOffsets[0];
  rawData.accel[1] = (int16_t)(tempData[2]<<8 | tempData[3]) * accelScale - accelOffsets[1];
  rawData.accel[2] = (int16_t)(tempData[4]<<8 | tempData[5]) * accelScale - accelOffsets[2];
  rawData.gyro[0] = (int16_t)(tempData[6]<<8 | tempData[7]) * gyroScale - gyroOffsets[0];
  rawData.gyro[1] = (int16_t)(tempData[8]<<8 | tempData[9]) * gyroScale - gyroOffsets[1];
  rawData.gyro[2] = (int16_t)(tempData[10]<<8 | tempData[11]) * gyroScale - gyroOffsets[2];
  rawData.temperature = (int16_t)(tempData[12]<<8 | tempData[13]) * tempScale - tempOffset;
  if(useMag){
    rawData.mag[0] = (int16_t)(tempData[15]<<8 | tempData[14]) * magScale;
    rawData.mag[1] = (int16_t)(tempData[17]<<8 | tempData[16]) * magScale;
    rawData.mag[2] = (int16_t)(tempData[19]<<8 | tempData[18]) * magScale;
  }
  rawData.timestamp = micros();
  return 0;
}

int ICM20948::init(int addr, AccelRange a_range, GyroRange g_range, AccelLowPassFilter a_filter, GyroLowPassFilter g_filter, bool useMag, bool initWire){
  //go init wire if necessary
  if(initWire){
    //init at highest speed
    Wire.begin();
    Wire.setClock(400000);
  }
  icmAddr = addr;
  this->useMag = useMag;
  //go try to read the who am i
  uint8_t id = 0;
  id = icmRead8(ICM20948_WHO_AM_I_REG);
  if(id != ICM20948_WHO_AM_I_EXPECTED_RESPONSE){
    if(id <= 0){
      return ICM20948_UNABLE_TO_CONNECT;
    }
    return ICM20948_WHO_AM_I_INCORRECT_RESPONSE;
  }
  //go and reset the microcontroller
  int resetStatus = reset();
  if(resetStatus < 0) return resetStatus;
  //otherwise set up for normal operation
  setSleepMode(false);
  setClockSource(CLKSEL_AUTO);
  setAccelRange(a_range);
  setGyroRange(g_range);
  setAccelLowPassFilter(a_filter);
  setGyroLowPassFilter(g_filter);
  if(useMag){
    int magStatus = initMagnetometer();
    if(magStatus != 0) return magStatus;
  }
  return 0;
}

int ICM20948::reset(){
  //set the reset bit, wait until it clears
  uint8_t base = icmRead8(ICM20948_PWR_MGMT_1_REG);
  base &= 0b01111111;
  base |= (1<<ICM20948_DEVICE_RESET_BIT);
  icmWrite8(ICM20948_PWR_MGMT_1_REG,base);
  //give a small delay for the reset
  delay(2);
  //Serial.println(icmRead8(ICM20948_PWR_MGMT_1_REG),BIN);
  //wait until its free
  bool done = false;
  long start = millis();
  while(!done && (millis() - start < 200)){
    //read it
    base = icmRead8(ICM20948_PWR_MGMT_1_REG);
    if((base & 0b10000000) == 0){
      done = true;
      //Serial.println(base,BIN);
    }
  }
  if(!done){
    //timeout occurred
    return ICM20948_RESET_TIMEOUT;
  }
  return 0;
}

bool ICM20948::checkForRawData(){
  return true;
  uint8_t base = icmRead8(ICM20948_DATA_RDY_REG);
  //Serial.println(base, BIN);
  base &= 0b00000001;
  if(base == 1) return true;
  return false;
}

int ICM20948::setGyroLowPassFilter(GyroLowPassFilter dlpf){
  //for now, just set it at 229 hz
  uint8_t base = icmRead8(ICM20948_GYRO_CONFIG_1_REG);
  //Serial.print("\n Orig Base: ");
  //Serial.println(base,BIN);
  //Serial.println("dlpf: " + String(dlpf));
  base &= 0b00000111;
  base |= ((uint8_t)dlpf << 3) | 1;
  //Serial.print("\n New Base: ");
  //Serial.println(base,BIN);
  return icmWrite8(ICM20948_GYRO_CONFIG_1_REG, base);
}

int ICM20948::setAccelLowPassFilter(AccelLowPassFilter dlpf){
  //for now, just set it at 229 hz
  uint8_t base = icmRead8(ICM20948_ACCEL_CONFIG_REG);
  //Serial.print("\n Orig Base: ");
  //Serial.println(base,BIN);
  //Serial.println("dlpf: " + String(dlpf));
  base &= 0b11000111;
  base |= ((uint8_t)dlpf << 3) | 1;
  //Serial.print("\n New Base: ");
  //Serial.println(base,BIN);
  return icmWrite8(ICM20948_ACCEL_CONFIG_REG, base);
}

int ICM20948::setAccelRange(AccelRange range){
  //set the sleep mode
//  setSleepMode(true);
  uint8_t base = icmRead8(ICM20948_ACCEL_CONFIG_REG); //
  base &= 0b11111001;
  base |= (range << ICM20948_ACCEL_FS_SEL_BIT);
  icmWrite8(ICM20948_ACCEL_CONFIG_REG, base);
  switch(range){
    case(ACCEL_2_GRAVITIES):
      accelScale = 1/16384.0;
      break;
    case(ACCEL_4_GRAVITIES):
      accelScale = 1/8192.0;
      break;
    case(ACCEL_8_GRAVITIES):
      accelScale = 1/4096.0;
      break;
    case(ACCEL_16_GRAVITIES):
      accelScale = 1/2048.0;
      break;
  }
  //setSleepMode(false);
  return 0;
}

int ICM20948::setGyroRange(GyroRange range){
  //set the range then update the scale factor
  uint8_t base = icmRead8(ICM20948_GYRO_CONFIG_1_REG); //in bank 2
  base &= 0b11111001;
  base |= range << ICM20948_GYRO_FS_SEL_BIT;
  icmWrite8(ICM20948_GYRO_CONFIG_1_REG, base); //in bank 2
  switch(range){
    case(GYRO_250_DPS):
      gyroScale = 1/131.0;
      break;
    case(GYRO_500_DPS):
      gyroScale = 1/65.5;
      break;
    case(GYRO_1000_DPS):
      gyroScale = 1/32.8;
      break;
    case(GYRO_2000_DPS):
      gyroScale = 1/16.4;
      break;
  }
  return 0;
}

int ICM20948::setSleepMode(bool shouldSleep){
  uint8_t base = icmRead8(ICM20948_PWR_MGMT_1_REG);
  base &= 0b10111111;
  if(shouldSleep) base |= (1 << ICM20948_SLEEP_BIT);
  icmWrite8(ICM20948_PWR_MGMT_1_REG, base);
  return base;
}

int ICM20948::setClockSource(ClockSource mode){
  uint8_t base = icmRead8(ICM20948_PWR_MGMT_1_REG);
  base &= 0b11111100;
  base |= mode;
  icmWrite8(ICM20948_PWR_MGMT_1_REG,base);
  return base;
}

int ICM20948::setStoredGyroOffsets(float *gx, float *gy, float *gz){
  //first scale into int16_t
  setGyroRange(GYRO_1000_DPS); //expects data to be stored in this scale/format
  int16_t x = (int16_t)(*gx/gyroScale + .5);
  int16_t y = (int16_t)(*gy/gyroScale + .5);
  int16_t z = (int16_t)(*gz/gyroScale + .5);
  //go write

  Wire.beginTransmission((byte)icmAddr);
  Wire.write(ICM20948_XG_OFFS_USRH_REG);
  //uint8_t xLow = (uint16_t)(x << 8) >> 8;
  //uint8_t xHigh = x >> 8;
  Wire.write(x >> 8);
  Wire.write((uint16_t)(x << 8) >> 8);
  Wire.write(y >> 8);
  Wire.write((uint16_t)(y << 8) >> 8);
  Wire.write(z >> 8);
  Wire.write((uint16_t)(z << 8) >> 8);
  Wire.endTransmission();
  return 0;
}

int ICM20948::setBank(uint8_t bank){
  if(bank != curBank){
    Wire.beginTransmission((byte)icmAddr);
    Wire.write(ICM20948_USER_BANK_SEL_REG);
    Wire.write(bank<<4);
    Wire.endTransmission();
    curBank = bank;
  }
  return 0;
}

uint8_t ICM20948::determineUserBank(uint16_t reg){
  //assume using the ICM20948_Registers registers!
  if(reg < USR_BANK_1_OFFSET) return 0;
  if(reg < USR_BANK_2_OFFSET) return 1;
  if(reg < USR_BANK_3_OFFSET) return 2;
  return 3;
}

int ICM20948::icmWrite(uint16_t startReg, uint8_t count, uint8_t *values){
  //figure out the bank
  if(icmAddr <= 0) return ICM20948_ADDR_NOT_SET;
  //go set the bank
  //Serial.println("Requested Register: " + String(startReg));
  int bank = determineUserBank(startReg);
  startReg -= bank << 8;
  Serial.println("Determined Bank: " + String(bank));
  Serial.println("Reg: " + String(startReg));
  Serial.println("Value: " + String(values[0]));
  //Serial.println("New RegisterValue: " + String(startReg));
  //while(true);
  setBank(bank);
  //carry on
  Wire.beginTransmission((byte)icmAddr);
  Wire.write(startReg);
  for(uint8_t i = 0; i < count; i ++){
    Wire.write(values[i]);
  }
  Wire.endTransmission();
  return 0;
}

int ICM20948::icmWrite8(uint16_t reg, uint8_t value){
  return icmWrite(reg, 1, &value);
}

int ICM20948::icmRead(uint16_t startReg, uint8_t count, uint8_t *values){
  //figure out the bank
  if(icmAddr <= 0) return ICM20948_ADDR_NOT_SET;
  //go set the bank
  int bank = determineUserBank(startReg);
  //Serial.println("Bank: " + String(bank));
  startReg -= bank << 8;
  setBank(bank);
  //carry on
  Wire.beginTransmission((byte)icmAddr);
  Wire.write(startReg);
  Wire.endTransmission();
  Wire.requestFrom((byte)icmAddr, count);
  for(uint8_t i = 0; i < count; i ++){
    values[i] = Wire.read();
  }
  return 0;
}

uint8_t ICM20948::icmRead8(uint16_t reg){
  uint8_t val  = 0;
  icmRead(reg, 1, &val);
  return val;
}


//magnetometer functions


int ICM20948::initMagnetometer(){
  //set the adress, do the appropriate, etc. etc.
  setSlaveI2CSpeed();
  enableI2CMaster();
  //setSlaveI2CSpeed();

  setSlaveAddr(ICM20948_MAGNETOMETER_ADDR);
  //resetMag();
  //setSlaveRead();
  uint8_t whoAmIResponse = readAk(0x01);
  Serial.println(whoAmIResponse);
  if(whoAmIResponse != ICM20948_MAGNETOMETER_EXPECTED_WHO_AM_I_RESPONSE) return ICM20498_MAGNETOMETER_WHO_AM_I_INCORRECT_RESPONSE;
  //go write to be continuous mode
  resetMag();
  //give it a while to reset!
  delay(100);
  setMagMode(MAGNETOMETER_CONTINUOUS_READ_MODE_4);
  //set up to place data in the ext sense data slots
  setSlaveRead();
  setSlaveStartRegister(ICM20948_MAGNETOMETER_XL_REG);
  setMagReadWriteSize(8);
  delay(10);
  return 0;
}

int ICM20948::getRawMagData(){
  //go check if data avail
  //if(!magCheckDataAvail()) return ICM20948_NO_NEW_MAG_DATA;
  //otherwise, lets read!
  uint8_t tempMagData[6];
  icmRead(ICM20948_EXT_SLV_SENS_DATA_00, 6, &tempMagData[0]);
  //otherwise, populate away!
  rawData.mag[0] = (int16_t)(tempMagData[0] | (tempMagData[1] << 8)) * magScale;
  rawData.mag[1] = (int16_t)(tempMagData[2] | (tempMagData[3] << 8)) * magScale;
  rawData.mag[2] = (int16_t)(tempMagData[4] | (tempMagData[5] << 8)) * magScale;
  rawData.timestamp = micros();
  return 0;
}

uint8_t ICM20948::writeAk(uint8_t reg, uint8_t value){
  setSlaveWrite();
  icmWrite8(ICM20948_I2C_SLV0_REG_REG,reg);
  icmWrite8(ICM20948_I2C_SLV0_D0_REG,value);
  icmWrite8(ICM20948_I2C_SLV0_CTRL_REG,1 << 7|1);
  //delay(10);
  Serial.print("\nValue of attempeted  Write: ");
  Serial.print(readAk(reg),BIN);
  Serial.print("\nWanted: ");
  Serial.println(value,BIN);
  return 0;
}


uint8_t ICM20948::readAk(uint8_t reg){
  //read through the slave mode
  //tell icm to read data
  //Serial.println(1 << 7 | ICM20948_MAGNETOMETER_ADDR);

  setSlaveRead();
  icmWrite8(ICM20948_I2C_SLV0_REG_REG,reg);
  icmWrite8(ICM20948_I2C_SLV0_CTRL_REG,1 << 7 | 1);
  delay(10);
  return icmRead8(ICM20948_EXT_SLV_SENS_DATA_00);
}

int ICM20948::setMagMode(MagnetometerMode mode){
  writeAk(ICM20948_MAGNETOMETER_CNTL_2_REG, mode);
  return 0;
}

int ICM20948::enableI2CMaster(){
  uint8_t base = icmRead8(ICM20948_USER_CTRL_REG);
  base &= 0b11011111;
  base |= 1 << 5;
  return icmWrite8(ICM20948_USER_CTRL_REG, 1 << 5);
}

int ICM20948::setSlaveI2CSpeed(){
  //default to 400 khz for now, a 7
  icmWrite8(ICM20948_I2C_MST_CTL_REG,0x07);
  return 0;
}

int ICM20948::setSlaveStartRegister(uint8_t reg){
  icmWrite8(ICM20948_I2C_SLV0_REG_REG,reg);
  return 0;
}

int ICM20948::setSlaveRead(){
  uint8_t base = icmRead8(ICM20948_I2C_SLV0_ADDR_REG);
  base &= 0b01111111;
  base |= 1 << 7;
  icmWrite8(ICM20948_I2C_SLV0_ADDR_REG, base);
  return 0;
}

int ICM20948::setSlaveWrite(){
  uint8_t base = icmRead8(ICM20948_I2C_SLV0_ADDR_REG);
  base &= 0b01111111;
  icmWrite8(ICM20948_I2C_SLV0_ADDR_REG, base);
  return 0;
}

int ICM20948::setSlaveAddr(uint8_t addr){
  icmWrite8(ICM20948_I2C_SLV0_ADDR_REG,addr);
  return 0;
}

int ICM20948::resetMag(){
  //reset the master i2c bus
  uint8_t base = icmRead8(ICM20948_USER_CTRL_REG);
  base &= 0b11111101;
  base |= 1 << 1;
  icmWrite8(ICM20948_USER_CTRL_REG, base);
  delay(1);
  writeAk(ICM20948_USER_CTRL_REG, 1); //reset
  delay(10);
  return 0;
}

int ICM20948::setMagReadWriteSize(uint8_t bytes){
  icmWrite8(ICM20948_I2C_SLV0_CTRL_REG,1 << 7 | bytes);
  return 0;
}
