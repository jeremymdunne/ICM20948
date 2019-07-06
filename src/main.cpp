#include <Arduino.h>
#include <ICM20948.h>

ICM20948 icm;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  Serial.println("Hello, WOrld!");
  int icm_init_status = icm.init(ICM20948_ADDR_AD0_HIGH, ICM20948::ACCEL_16_GRAVITIES, ICM20948::GYRO_2000_DPS, ICM20948::ACCEL_DPLF_50_HZ, ICM20948::GYRO_DLPF_73_HZ, true);
  if(icm_init_status < 0){
    Serial.println("ICM failed to init: " + String(icm_init_status));
    while(true);
  }
  Serial.println("ICM init success!");
  //icm.setGyroRange(ICM20948::GYRO_250_DPS);
  //icm.setAccelRange(ICM20948::ACCEL_2_GRAVITIES);
  //icm.setAccelLowPassFilter(ICM20948::ACCEL_DLPF_6_HZ);
  //icm.setGyroLowPassFilter(ICM20948::GYRO_DLPF_9_HZ);
}

ICM20948::RawData icmData;
long lastUpdate = 0;
void loop() {
  long start = micros();
  icm.getData(&icmData);
  long stop = micros();
  Serial.println("Accel: " + String(icmData.accel[0]) + "\t" + String(icmData.accel[1]) + "\t" + String(icmData.accel[2]));
  Serial.println("Gyro: " + String(icmData.gyro[0]) + "\t" + String(icmData.gyro[1]) + "\t" + String(icmData.gyro[2]));
  Serial.println("Temp: " + String(icmData.temperature));
  Serial.println("Mag: " + String(icmData.mag[0]) + "\t" + String(icmData.mag[1]) + "\t" + String(icmData.mag[2]));
  Serial.println("Time: " + String(stop-start));
  delay(50);
}
