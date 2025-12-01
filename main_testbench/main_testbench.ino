#include <TeensyThreads.h>
#include <SPI.h>
#include <BMI088.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SD.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>

void sensorTask();

void setup() {
  // 1) Start Serial at 115200
  Serial.begin(115200);
  delay(200);                   // give Serial a moment to wake up
  Serial.println("Serial is up");
  /*
  if (!SD.begin(BUILTIN_SDCARD)) {
    //Serial.println("SD init failed!");
    while (1);
  }

  // SD Card initialization
  pinMode(BARO_CS,  OUTPUT); digitalWrite(BARO_CS,  HIGH);
  pinMode(ACCEL_CS, OUTPUT); digitalWrite(ACCEL_CS, HIGH);
  pinMode(GYRO_CS,  OUTPUT); digitalWrite(GYRO_CS,  HIGH);
  pinMode(MAG_CS,   OUTPUT); digitalWrite(MAG_CS,   HIGH);
  SPI.begin();
  
  delay(50);
  resetBaro();    delay(10);
  readBaroCalibration(); delay(10);
  if (accel.begin() < 0) {
    //Serial.println("Accel init error"); 
    while (1);
  }
  if (gyro.begin()  < 0) {
    //Serial.println("Gyro init error");  
    while (1);
  }
  magWriteReg(MAG_CFG_C, 0x34);
  magWriteReg(MAG_CFG_A, 0x8C);
  */
  // 2) Configure the thread tick interval
  threads.setSliceMillis(5);

  // 3) Launch ONLY sensorTask with a moderate (2 KB) stack to begin
  if (threads.addThread(sensorTask, nullptr, 2048) < 0) {
    Serial.println("ERROR: could not start sensorTask");
    while (1) { /* hang */ }
  }
}

void loop() {
  // nothing here—sensorTask() drives everything
}

void sensorTask() {
  while (1) {
    // Print a fixed string so we know this thread is alive
    Serial.println("sensorTask alive!");
    threads.delay(500);
  }
}
