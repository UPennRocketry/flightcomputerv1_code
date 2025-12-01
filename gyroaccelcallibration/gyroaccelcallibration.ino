#include <SPI.h>
#include "BMI088.h"

// —————— CHIP SELECT PINS ——————
const uint8_t BARO_CS  = 10;
const uint8_t ACCEL_CS = 37;
const uint8_t GYRO_CS  = 36;
const uint8_t MAG_CS   = 38;

// —————— SPI SETTINGS ——————
SPISettings baroSPI(20000000, MSBFIRST, SPI_MODE0);
SPISettings imuSPI (10000000, MSBFIRST, SPI_MODE0);
SPISettings magSPI (10000000, MSBFIRST, SPI_MODE0);

// —————— BARO CALIBRATION STORAGE ——————
uint16_t C[8];

// —————— BARO HELPER FUNCTIONS ——————
void resetBaro() {
  SPI.beginTransaction(baroSPI);
    digitalWrite(BARO_CS, LOW);
    SPI.transfer(0x1E);
    digitalWrite(BARO_CS, HIGH);
  SPI.endTransaction();
}

void readBaroCalibration() {
  for (uint8_t i = 0; i < 8; i++) {
    SPI.beginTransaction(baroSPI);
      digitalWrite(BARO_CS, LOW);
      SPI.transfer(0xA0 | (i << 1));
      uint8_t hi = SPI.transfer(0x00);
      uint8_t lo = SPI.transfer(0x00);
      C[i] = (hi << 8) | lo;
      digitalWrite(BARO_CS, HIGH);
    SPI.endTransaction();
  }
}

uint32_t readADCBaro(uint8_t cmd, uint16_t delayMs) {
  uint32_t val = 0;
  SPI.beginTransaction(baroSPI);
    digitalWrite(BARO_CS, LOW);
    SPI.transfer(cmd);
    digitalWrite(BARO_CS, HIGH);
  SPI.endTransaction();
  delay(delayMs);
  SPI.beginTransaction(baroSPI);
    digitalWrite(BARO_CS, LOW);
      SPI.transfer(0x00); val  = (uint32_t)SPI.transfer(0x00) << 16;
      val |= (uint32_t)SPI.transfer(0x00) << 8;
      val |=  (uint32_t)SPI.transfer(0x00);
      digitalWrite(BARO_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void readBarometerRaw(uint32_t &D1, uint32_t &D2) {
  D2 = readADCBaro(0x58, 10);  // temperature raw
  D1 = readADCBaro(0x48, 10);  // pressure raw
}

// —————— MAG HELPER FUNCTIONS ——————
uint8_t magReadReg(uint8_t reg) {
  SPI.beginTransaction(magSPI);
    digitalWrite(MAG_CS, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(MAG_CS, HIGH);
  SPI.endTransaction();
  return v;
}

void magWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(magSPI);
    digitalWrite(MAG_CS, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(MAG_CS, HIGH);
  SPI.endTransaction();
}

// —————— SENSOR OBJECTS ——————
Bmi088Accel accel(SPI, ACCEL_CS);
Bmi088Gyro  gyro(SPI, GYRO_CS);


// storage for computed biases
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;

const int   CAL_SAMPLES = 1000;    // how many readings to average
const int   CAL_DELAY   = 5;      // ms between samples

void calibrateGyro() {
  double sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    accel.readSensor();
    gyro.readSensor();

    sumX += gyro.getGyroX_rads();
    sumY += gyro.getGyroY_rads();
    sumZ += gyro.getGyroZ_rads();

    delay(CAL_DELAY);
  }

  gyroBiasX = sumX / CAL_SAMPLES;
  gyroBiasY = sumY / CAL_SAMPLES;
  gyroBiasZ = sumZ / CAL_SAMPLES;

  Serial.print("Gyro biases (rad/s): ");
  Serial.print(gyroBiasX, 6); Serial.print(", ");
  Serial.print(gyroBiasY, 6); Serial.print(", ");
  Serial.println(gyroBiasZ, 6);
}

void calibrateAccel() {
  double sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    accel.readSensor();
    gyro.readSensor();

    sumX += accel.getAccelX_mss();
    sumY += accel.getAccelY_mss();
    sumZ += accel.getAccelZ_mss();

    delay(CAL_DELAY);
  }

  accelBiasX = sumX / CAL_SAMPLES;
  accelBiasY = sumY / CAL_SAMPLES;
  accelBiasZ = (sumZ / CAL_SAMPLES) + 9.80665;  
  // subtract 1 g on Z so bias is zero when stationary

  Serial.print("Accel biases (m/s²): ");
  Serial.print(accelBiasX, 6); Serial.print(", ");
  Serial.print(accelBiasY, 6); Serial.print(", ");
  Serial.println(accelBiasZ, 6);
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  // CS pins
  pinMode(BARO_CS,  OUTPUT); digitalWrite(BARO_CS,  HIGH);
  pinMode(ACCEL_CS, OUTPUT); digitalWrite(ACCEL_CS, HIGH);
  pinMode(GYRO_CS,  OUTPUT); digitalWrite(GYRO_CS,  HIGH);
  pinMode(MAG_CS,   OUTPUT); digitalWrite(MAG_CS,   HIGH);

  // SPI init
  SPI.begin();
  delay(50);

  // Barometer init
  resetBaro();
  delay(10);
  readBaroCalibration();
  delay(10);

  // IMU init
  if (accel.begin() < 0) { Serial.println("Accel init error"); while(1); }
  if (gyro.begin()  < 0) { Serial.println("Gyro init error");  while(1); }

  // Mag init (LIS2MDL)
  magWriteReg(0x62, 0x34);
  magWriteReg(0x60, 0x8C);
  if (magReadReg(0x4F) != 0x40) {
    Serial.println("Mag init error");
    while(1);
  }

  //============================== bias ===============================
  calibrateGyro();
  calibrateAccel();
  Serial.println("done");
}

void loop() {
  // --- Read raw barometer ADC values ---
  uint32_t D1, D2;
  readBarometerRaw(D1, D2);

  // --- Read IMU ---
  accel.readSensor();
  gyro.readSensor();
  float gx = gyro.getGyroX_rads() - gyroBiasX;
  float gy = gyro.getGyroY_rads() - gyroBiasY;
  float gz = gyro.getGyroZ_rads() - gyroBiasZ;

  float ax = accel.getAccelX_mss() - accelBiasX;
  float ay = accel.getAccelY_mss() - accelBiasY;
  float az = accel.getAccelZ_mss() - accelBiasZ;

  /*float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();
  float gx = gyro.getGyroX_rads();
  float gy = gyro.getGyroY_rads();
  float gz = gyro.getGyroZ_rads();*/

  // --- Read magnetometer raw registers ---
  while (!(magReadReg(0x67) & 0x08));  // wait for data ready
  int16_t mX = (int16_t)(magReadReg(0x69) << 8 | magReadReg(0x68));
  int16_t mY = (int16_t)(magReadReg(0x6B) << 8 | magReadReg(0x6A));
  int16_t mZ = (int16_t)(magReadReg(0x6D) << 8 | magReadReg(0x6C));

  // --- Print everything as CSV ---
  //Serial.print(D2); Serial.print(",");  // baro temp ADC
  //Serial.print(D1); Serial.print(",");  // baro pressure ADC

  Serial.print(ax, 5); Serial.print(",");
  Serial.print(ay, 5); Serial.print(",");
  Serial.print(az, 5); Serial.print(",");

  Serial.print(gx, 5); Serial.print(",");
  Serial.print(gy, 5); Serial.print(",");
  Serial.println(gz, 5);

  //Serial.print(mX); Serial.print(",");
  //Serial.print(mY); Serial.print(",");
  //Serial.println(mZ);

  delay(100);  // adjust sample rate as needed
}
