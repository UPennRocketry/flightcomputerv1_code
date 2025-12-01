#include <SPI.h>
#include <SD.h>
#include "BMI088.h"
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <RH_RF95.h> // uninstall radiohead library when uploading bc it uses the teensy RH_RF95
#include <RHHardwareSPI1.h>
#include <math.h>

// —————— SENSOR (baro, IMU, mag) SETUP ——————

// Chip select pins
const uint8_t BARO_CS  = 10;
const uint8_t ACCEL_CS = 37;
const uint8_t GYRO_CS  = 36;
const uint8_t MAG_CS   = 38;

// SPI settings
SPISettings baroSPI(20000000, MSBFIRST, SPI_MODE0);  // MS5607 up to 20 MHz
SPISettings imuSPI (10000000, MSBFIRST, SPI_MODE0);  // BMI088 up to 10 MHz
SPISettings magSPI (10000000, MSBFIRST, SPI_MODE0);  // LIS2MDL up to 10 MHz

// MS5607 calibration storage
uint16_t C[8];  // coefficients C[1]…C[6]

// MS5607 low‐level helpers
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
      SPI.transfer(0x00);
      val  = (uint32_t)SPI.transfer(0x00) << 16;
      val |= (uint32_t)SPI.transfer(0x00) << 8;
      val |=  (uint32_t)SPI.transfer(0x00);
      digitalWrite(BARO_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void readBarometer(float &temperature, float &pressure) {
  uint32_t D2 = readADCBaro(0x58, 10);
  double dT   = (double)D2 - ((double)C[5] * 256.0);
  double TEMP = 2000.0 + dT * ((double)C[6] / 8388608.0);
  double OFF  = (double)C[2] * 131072.0 + ((double)C[4] * dT) / 64.0;
  double SENS = (double)C[1] * 65536.0  + ((double)C[3] * dT) / 128.0;
  uint32_t D1 = readADCBaro(0x48, 10);
  double P    = (((double)D1 * SENS) / 2097152.0 - OFF) / (32768.0); // Pa, /100 for mbar
  temperature = TEMP / 100.0;
  pressure    = P;
}

// Magnetometer registers
#define MAG_WHO_AM_I   0x4F
#define MAG_CFG_A      0x60
#define MAG_CFG_B      0x61
#define MAG_CFG_C      0x62
#define MAG_STATUS     0x67
#define MAG_OUTX_L     0x68

// Magnetometer helpers
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

#define LAPSE_RATE           0.0065      // K/m, standard tropospheric lapse
#define GRAVITY              9.80665     // m/s^2
#define MOLAR_MASS           0.0289644   // kg/mol (dry air)
#define UNIVERSAL_GAS_CONST  8.3144598   // J/(mol·K)
#define SEA_LEVEL_P          101325.00   // Pa
float baseT, baseP; // refT and refP

double calculateAltitudeMeters(double currentPressure, double referencePressure, double referenceTemperature) {
    double exponent = (UNIVERSAL_GAS_CONST * LAPSE_RATE) / (GRAVITY * MOLAR_MASS);
    double ratio = currentPressure / referencePressure;
    if (ratio <= 0.0) {
        ratio = 1e-6;
    } 
    double term = pow(ratio, exponent);
    double altitudeMeters = (referenceTemperature / LAPSE_RATE) * (1.0 - term);
    return altitudeMeters;
}

// Create BMI088 objects
Bmi088Accel accel(SPI, ACCEL_CS);
Bmi088Gyro  gyro(SPI, GYRO_CS);

// —————— GPS SETUP ——————
SFE_UBLOX_GNSS myGNSS;

// —————— RADIO SETUP ——————
// RFM95W-915S2 connections to Teensy 4.1
#define RFM95_MISO  1    // SPI1 MISO
#define RFM95_MOSI  26   // SPI1 MOSI
#define RFM95_SCK   27   // SPI1 SCK
#define RFM95_CS    28   // Chip select
#define RFM95_RST   0    // Reset line
#define RFM95_INT   29   // DIO0 → IRQ for RX/TX done

// Instantiate the driver on SPI1
// The third parameter picks SPI1 (hardware_spi1) instead of the default SPI port
RH_RF95 rf95(RFM95_CS, RFM95_INT, hardware_spi1);

// —————— LOG FILES ——————
File sensorLogFile;
char sensorLogName[16];
File gpsLogFile;
char gpsLogName[16];
//File dataLogFile;
//char dataLogName[16];

File openNumberedLog(const char *baseName, const char *ext = ".csv") {
  char  fname[16];
  uint8_t i = 0;

  // keep trying names until we find one that doesn't exist
  while (true) {
    if (i == 0) {
      // first try “datalog.csv”
      snprintf(fname, sizeof(fname), "%s%s", baseName, ext);
  } else {
    // then “datalog1.csv”, “datalog2.csv”, …
    snprintf(fname, sizeof(fname), "%s%u%s", baseName, i, ext);
  }
    if (! SD.exists(fname)) {
      // found a free name
      File f = SD.open(fname, FILE_WRITE | O_TRUNC);
      if (f) {
        //Serial.print("Logging to: ");
        //Serial.println(fname);
        return f;
      } else {
        //Serial.print("Failed to open ");
        //Serial.println(fname);
        return File();  // invalid File
      }
    }
    i++;
    // give up after e.g. 255 tries
    if (i == 255) {
      //Serial.println("Too many log files!");
      return File();
    }
  }
}

void setup() {
  pinMode(2, OUTPUT); // LED
  pinMode(40, OUTPUT); // PYRO1
  pinMode(41, OUTPUT); // PYRO2
  digitalWrite(2, HIGH);
  digitalWrite(40, LOW);
  digitalWrite(41, LOW);

  // — Serial for debugging —
  Serial.begin(115200);
  while (!Serial);

  // — SD CARD INITIALIZATION —
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // — SENSOR (baro, IMU, mag) INITIALIZATION —
  pinMode(BARO_CS,  OUTPUT); digitalWrite(BARO_CS,  HIGH);
  pinMode(ACCEL_CS, OUTPUT); digitalWrite(ACCEL_CS, HIGH);
  pinMode(GYRO_CS,  OUTPUT); digitalWrite(GYRO_CS,  HIGH);
  pinMode(MAG_CS,   OUTPUT); digitalWrite(MAG_CS,   HIGH);

  SPI.begin();
  delay(50);

  resetBaro();
  delay(10);
  readBaroCalibration();
  delay(10);

  if (accel.begin() < 0) {
    Serial.println("Accel init error");
    while (1);
  }
  if (gyro.begin()  < 0) {
    Serial.println("Gyro init error");
    while (1);
  }

  magWriteReg(MAG_CFG_C, 0x34);
  magWriteReg(MAG_CFG_B, 0x0C);
  magWriteReg(MAG_CFG_A, 0x8C);
  uint8_t mid = magReadReg(MAG_WHO_AM_I);
  Serial.print("MAG WHO_AM_I=0x");
  Serial.println(mid, HEX);
  if (mid != 0x40) {
    Serial.println("Mag init error");
    while (1);
  }

  // BASE T AND P
  readBarometer(baseT, baseP);
    
  sensorLogFile = openNumberedLog("sensorlog");
  strncpy(sensorLogName, sensorLogFile.name(), sizeof(sensorLogName));
  if (!sensorLogFile) {
    Serial.println("Failed to open sensor log"); 
    while (1);
  }
  //sensorLogFile.print("BaseT_C,");
  //sensorLogFile.println(baseT, 2);
  //sensorLogFile.print("BaseP_Pa,");
  //sensorLogFile.println(baseP, 2);
  sensorLogFile.println("Time_ms,T_C,P_Pa,Alt_m,AccX_mss,AccY_mss,AccZ_mss,"
            "GyroX_rads,GyroY_rads,GyroZ_rads,"
            "MagX_raw,MagY_raw,MagZ_raw,PYRO");
  sensorLogFile.close();
  gpsLogFile = openNumberedLog("gpslog");
  strncpy(gpsLogName, gpsLogFile.name(), sizeof(gpsLogName));
  if (!gpsLogFile) {
    Serial.println("Failed to open gps log"); 
    while (1);
  }
  gpsLogFile.println("Time_ms,Lat,Long,Alt_m,Fix,Sats");
  gpsLogFile.close();
  /*
  dataLogFile = openNumberedLog("datalog");
  strncpy(dataLogName, dataLogFile.name(), sizeof(dataLogName));
  if (!dataLogFile) {
    Serial.println("Failed to open data log"); 
    while (1);
  }
  dataLogFile.println("Time_ms,T_C,P_mbar,AccX_mss,AccY_mss,AccZ_mss,"
            "GyroX_rads,GyroY_rads,GyroZ_rads,"
            "MagX_raw,MagY_raw,MagZ_raw,TempAcc_C,"
            "Time_ms,Lat,Long,Alt_m,Fix,Sats");
  dataLogFile.close();
  */
  
  // — GPS INITIALIZATION — GPS IS BROKEN, NEED TO RESOLDER
  /*
  Wire.begin();
  while (!myGNSS.begin()) {
    Serial.println("GNSS not detected. Check wiring.");
    delay(10);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);         // Use UBX protocol only
  myGNSS.setNavigationFrequency(10);          // 10 Hz updates
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save I/O settings
  Serial.println("GNSS initialized.");
  */
  // — RADIO INITIALIZATION —
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  SPI1.begin();  // Init the SPI1 bus (Teensy core maps SPI1 to pins 1,26,27)

  if (!rf95.init()) {
    //Serial.println("RFM95 init failed");
    while (1);
  }
  //Serial.println("RFM95 init OK");

  if (!rf95.setFrequency(915.0)) {
    //Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.println("Frequency set to 915 MHz");

  rf95.setTxPower(23, false);
  digitalWrite(2, LOW);
}

unsigned long timebuf[10] = {0.0};
double altitudebuf[10] = {0.0};
double velocitybuf[10] = {0.0};
int idx = 0;
double altsum = 0;
double velsum = 0;

void rollingAverage(double newalt, unsigned long t) {
  double delta = 10000000;
  if (t - timebuf[idx] > 0) {
    delta = t - timebuf[idx];
  }
  altsum -= altitudebuf[idx];
  velsum -= velocitybuf[idx];
  //Serial.println(newalt - altitudebuf[idx]);
  //Serial.println(delta / 1000);
  velocitybuf[idx] = (newalt - altitudebuf[idx]) / (delta / 1000);
  //Serial.println(velocitybuf[idx]);
  altitudebuf[idx] = newalt;
  timebuf[idx] = t;
  
  altsum += altitudebuf[idx];
  velsum += velocitybuf[idx];
  idx = (idx + 1) % 10;
}

bool DESCENT = false;
unsigned long pyrotime = -1;
bool parachutedeploy = false;

void loop() {
  // —————— 1) READ SENSOR DATA ——————
  float T, P;
  readBarometer(T, P);

  accel.readSensor();
  gyro.readSensor();
  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();
  float gx = gyro.getGyroX_rads();
  float gy = gyro.getGyroY_rads();
  float gz = gyro.getGyroZ_rads();

  while (!(magReadReg(MAG_STATUS) & 0x08));
  uint8_t xl = magReadReg(MAG_OUTX_L);
  uint8_t xh = magReadReg(MAG_OUTX_L + 1);
  uint8_t yl = magReadReg(MAG_OUTX_L + 2);
  uint8_t yh = magReadReg(MAG_OUTX_L + 3);
  uint8_t zl = magReadReg(MAG_OUTX_L + 4);
  uint8_t zh = magReadReg(MAG_OUTX_L + 5);
  int16_t mX = (int16_t)(xh << 8 | xl);
  int16_t mY = (int16_t)(yh << 8 | yl);
  int16_t mZ = (int16_t)(zh << 8 | zl);
  double alt = calculateAltitudeMeters(P, baseP, baseT+273.15);
  // ^ Use baseP for RELATIVE altitude. Use SEA_LEVEL_P for TRUE altitude
  // For relative altitude, find elevation at logitude/latitude of launch site and add to get sea level altitude

  unsigned long t = millis();
  // Build a CSV‐style string of sensor data
  String sensorData = String(t) + ","
                    + String(T, 5) + ","
                    + String(P, 5) + ","
                    + String(alt, 5) + ","
                    + String(ax, 5) + ","
                    + String(ay, 5) + ","
                    + String(az, 5) + ","
                    + String(gx, 5) + ","
                    + String(gy, 5) + ","
                    + String(gz, 5) + ","
                    + String(mX) + ","
                    + String(mY) + ","
                    + String(mZ);
  //Serial.println(sensorData);

  // CHECK PYRO
  rollingAverage(alt, t);
  double avgalt = altsum / 10;
  double avgvel = velsum / 10;
  //Serial.println(avgalt);
  //Serial.println(avgvel);
  if (avgalt > 1000 && avgvel < -10) {
    DESCENT = true;
  }
  if (DESCENT && !parachutedeploy && avgalt < 150) {
    digitalWrite(40, HIGH);
    digitalWrite(41, HIGH);
    pyrotime = millis();
    parachutedeploy = true;
    
    sensorLogFile = SD.open(sensorLogName, FILE_WRITE); 
    if (sensorLogFile) {
      String p = String(pyrotime) + ", PYRO FIRED";
      sensorLogFile.println(p);
      sensorLogFile.close();
    } else {
      //Serial.println("Error opening sensor log for append");
    }
  }
  if (parachutedeploy && millis() - pyrotime > 5000) {
    digitalWrite(40, LOW);
    digitalWrite(41, LOW);
  }

  // —————— 2) ATTEMPT GPS READ ——————
  String gpsData = "";  // default to empty if no lock

/*// IREC 2025 NO GPS ANTENNA SO BLOCKING THIS OUT
  if (myGNSS.getPVT()) {
    float lat = myGNSS.getLatitude()   / 10000000.0;
    float lon = myGNSS.getLongitude()  / 10000000.0;
    float alt = myGNSS.getAltitudeMSL() / 1000.0;
    uint8_t fix  = myGNSS.getFixType();
    uint8_t sats = myGNSS.getSIV();

    if (fix > 0) {
      gpsData = String(millis()) + ","
              + String(lat, 6) + ","
              + String(lon, 6) + ","
              + String(alt) + ","
              + String(fix) + ","
              + String(sats);
    }
  }
*/
  // —————— 3) LOG TO SD CARD ——————
  
  // Log sensor data every loop
  sensorLogFile = SD.open(sensorLogName, FILE_WRITE); 
  if (sensorLogFile) {
    sensorLogFile.println(sensorData);
    sensorLogFile.close();
  } else {
    //Serial.println("Error opening sensor log for append");
    //return;
  }

/*// IREC 2025 NO GPS ANTENNA SO BLOCKING THIS OUT

  // Log GPS data only if locked
  if (gpsData.length() > 0) {
    gpsLogFile = SD.open(gpsLogName, FILE_WRITE); 
    if (gpsLogFile) {
      String data = gpsData;
      gpsLogFile.println(gpsData);
      gpsLogFile.close();
    } else {
      //Serial.println("Error opening gps log for append");
      return;
    }
  }
*/
  
  /*
  dataLogFile = SD.open(dataLogName, FILE_WRITE); 
  if (dataLogFile) {
    String data = sensorData + "," + gpsData;
    dataLogFile.println(data);
    dataLogFile.close();
  } else {
    Serial.println("Error opening data log for append");
  }
  */
  
  // —————— 4) SEND OVER RADIO ——————
  // Send sensor string
  //Serial.println(rf95.waitPacketSent(0));
  if (rf95.mode() == RHGenericDriver::RHModeIdle) {
    String payload = "SENSE: " + sensorData + "\nGPS: " + gpsData;  // or use '\n' or any delimiter you like
    int len = payload.length();
    if (len >= RH_RF95_MAX_MESSAGE_LEN) {
      // if you exceed max, either truncate or split into multiple packets
      len = RH_RF95_MAX_MESSAGE_LEN - 1;
      payload = payload.substring(0, len);
    }
    char buf[len + 1];
    payload.toCharArray(buf, len + 1);
    rf95.send((uint8_t*)buf, len);
  }
  // —————— 5) PRINT TO SERIAL FOR DEBUGGING ——————
  /*
  Serial.println("SENSOR: " + sensorData);
  if (gpsData.length() > 0) {
    Serial.println("GPS:    " + gpsData);
  } else {
    Serial.println("GPS:    <no lock>");
  }
  */
}
