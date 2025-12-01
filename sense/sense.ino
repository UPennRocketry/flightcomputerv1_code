#include <SPI.h>
#include <SD.h>
#include "BMI088.h"

// —— Chip select pins ——
const uint8_t BARO_CS  = 10;
const uint8_t ACCEL_CS = 37;
const uint8_t GYRO_CS  = 36;
const uint8_t MAG_CS   = 38;

// —— SPI settings ——
SPISettings baroSPI(20000000, MSBFIRST, SPI_MODE0);  // MS5607 up to 20 MHz
SPISettings imuSPI (10000000, MSBFIRST, SPI_MODE0);  // BMI088 up to 10 MHz
SPISettings magSPI (10000000, MSBFIRST, SPI_MODE0);  // LIS2MDL up to 10 MHz

// —— MS5607 calibration storage ——
uint16_t C[8];  // coefficients C[1]…C[6]

// —— MS5607 low‑level helpers ——
void resetBaro() {
  SPI.beginTransaction(baroSPI);
    digitalWrite(BARO_CS, LOW); SPI.transfer(0x1E); digitalWrite(BARO_CS, HIGH);
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
    delay(1);
  }
}

uint32_t readADCBaro(uint8_t cmd, uint16_t delayMs) {
  uint32_t val = 0;
  SPI.beginTransaction(baroSPI);
    digitalWrite(BARO_CS, LOW); SPI.transfer(cmd); digitalWrite(BARO_CS, HIGH);
  SPI.endTransaction();
  delay(delayMs);
  SPI.beginTransaction(baroSPI);
    digitalWrite(BARO_CS, LOW);
      SPI.transfer(0x00);
      val  = (uint32_t)SPI.transfer(0x00) << 16;
      val |= (uint32_t)SPI.transfer(0x00) << 8;
      val |=  (uint32_t)SPI.transfer(0x00);
      digitalWrite(BARO_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void readBarometer(float &temperature, float &pressure) {
  uint32_t D1 = readADCBaro(0x48, 10);
  uint32_t D2 = readADCBaro(0x58, 10);
  double dT   = (double)D2 - ((double)C[5] * 256.0);
  double TEMP = 2000.0 + dT * ((double)C[6] / 8388608.0);
  double OFF  = (double)C[2] * 131072.0 + ((double)C[4] * dT) / 64.0;
  double SENS = (double)C[1] * 65536.0  + ((double)C[3] * dT) / 128.0;
  double P    = (((double)D1 * SENS) / 2097152.0 - OFF) / (32768.0 * 100);
  temperature = TEMP / 100.0;
  pressure    = P;
}

// —— Magnetometer registers ——
#define MAG_WHO_AM_I   0x4F
#define MAG_CFG_A      0x60
#define MAG_CFG_C      0x62
#define MAG_STATUS     0x67
#define MAG_OUTX_L     0x68

// —— Magnetometer helpers ——
void magWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(magSPI);
  digitalWrite(MAG_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(MAG_CS, HIGH);
  SPI.endTransaction();
}

uint8_t magReadReg(uint8_t reg) {
  SPI.beginTransaction(magSPI);
  digitalWrite(MAG_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(MAG_CS, HIGH);
  SPI.endTransaction();
  return v;
}

// —— Create BMI088 objects ——
Bmi088Accel accel(SPI, ACCEL_CS);
Bmi088Gyro  gyro(SPI, GYRO_CS);

// —— Data logging file ——
File logFile;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  
  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD init failed!"); while (1);
  }
  logFile = SD.open("datalog.csv", O_WRITE | O_TRUNC); //OVERWRITES file, to append use FILE_WRITE instead
  if (!logFile) {
    Serial.println("Failed to open datalog.csv"); while (1);
  }
  // CSV header
  logFile.println("Time_ms,T_C,P_mbar,AccX_mss,AccY_mss,AccZ_mss,GyroX_rads,GyroY_rads,GyroZ_rads,MagX_raw,MagY_raw,MagZ_raw,TempAcc_C");
  

  // Configure CS pins
  pinMode(BARO_CS,  OUTPUT); digitalWrite(BARO_CS,  HIGH);
  pinMode(ACCEL_CS, OUTPUT); digitalWrite(ACCEL_CS, HIGH);
  pinMode(GYRO_CS,  OUTPUT); digitalWrite(GYRO_CS,  HIGH);
  pinMode(MAG_CS,   OUTPUT); digitalWrite(MAG_CS,   HIGH);

  // Start SPI
  SPI.begin();
  delay(50);

  // Initialize sensors
  resetBaro();    delay(10);
  readBaroCalibration(); delay(10);
  if (accel.begin() < 0) { Serial.println("Accel init error"); while (1); }
  if (gyro.begin()  < 0) { Serial.println("Gyro init error");  while (1); }
  // Magnetometer init
  magWriteReg(MAG_CFG_C, 0x34);
  magWriteReg(MAG_CFG_A, 0x8C);
  uint8_t mid = magReadReg(MAG_WHO_AM_I);
  Serial.print("MAG WHO_AM_I=0x"); Serial.println(mid, HEX);
  if (mid != 0x40) { Serial.println("Mag init error"); while (1); }
}

void loop() {
  unsigned long t = millis();
  // Baro
  float T, P;
  readBarometer(T, P);
  // IMU
  accel.readSensor();
  gyro.readSensor();
  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();
  float gx = gyro.getGyroX_rads();
  float gy = gyro.getGyroY_rads();
  float gz = gyro.getGyroZ_rads();
  // Mag
  while (!(magReadReg(MAG_STATUS) & 0x08));
  uint8_t xl = magReadReg(MAG_OUTX_L);
  uint8_t xh = magReadReg(MAG_OUTX_L+1);
  uint8_t yl = magReadReg(MAG_OUTX_L+2);
  uint8_t yh = magReadReg(MAG_OUTX_L+3);
  uint8_t zl = magReadReg(MAG_OUTX_L+4);
  uint8_t zh = magReadReg(MAG_OUTX_L+5);
  int16_t mX = (int16_t)(xh<<8 | xl);
  int16_t mY = (int16_t)(yh<<8 | yl);
  int16_t mZ = (int16_t)(zh<<8 | zl);

  // Print to Serial
  Serial.print("T_BARO[°C]: "); Serial.print(T,2);
  Serial.print(", T_ACCEL[°C]: "); Serial.print(accel.getTemperature_C(),2);
  Serial.print(", P[mbar]: "); Serial.print(P,2);
  Serial.print(", ACC[m/s²]: "); Serial.print(ax,3); Serial.print(","); Serial.print(ay,3); Serial.print(","); Serial.print(az,3);
  Serial.print(", GYRO[rad/s]: "); Serial.print(gx,3); Serial.print(","); Serial.print(gy,3); Serial.print(","); Serial.print(gz,3);
  Serial.print(", MAG[raw]: "); Serial.print(mX); Serial.print(","); Serial.print(mY); Serial.print(","); Serial.println(mZ);

  // Log to SD
  
  logFile.print(t);        logFile.print(',');
  logFile.print(T,2);     logFile.print(',');
  logFile.print(P,2);     logFile.print(',');
  logFile.print(ax,3);    logFile.print(',');
  logFile.print(ay,3);    logFile.print(',');
  logFile.print(az,3);    logFile.print(',');
  logFile.print(gx,3);    logFile.print(',');
  logFile.print(gy,3);    logFile.print(',');
  logFile.print(gz,3);    logFile.print(',');
  logFile.print(mX);      logFile.print(',');
  logFile.print(mY);      logFile.print(',');
  logFile.print(mZ);      logFile.print(',');
  logFile.println(accel.getTemperature_C(),2);
  logFile.flush();
  
  delay(100);
}
