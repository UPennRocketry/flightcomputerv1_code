#include <SPI.h>

// pin assignments
const uint8_t CSB_PIN = 10;    // chip‑select (active LOW)
const SPISettings ms5607SPI(20000000, MSBFIRST, SPI_MODE0);

// storage for the 8 PROM words (we only use C[1]…C[6])
uint16_t C[8];

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  // configure CSB
  pinMode(CSB_PIN, OUTPUT);
  digitalWrite(CSB_PIN, HIGH);

  // init SPI bus (uses pins 11=MOSI,12=MISO,13=SCK)
  SPI.begin();

  // reset sensor and read calibration
  resetSensor();
  delay(10);
  readCalibration();
}

void loop() {
  // read raw pressure D1 and temperature D2 (OSR=4096)
  uint32_t D1 = readADC(0x48, 10);  // Convert D1, OSR=4096 (typ 7.40 ms → delay 10 ms) :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}
  uint32_t D2 = readADC(0x58, 10);  // Convert D2, OSR=4096 :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}

  Serial.print("D1 (raw pressure) = "); Serial.println(D1);
  Serial.print("D2 (raw temp)     = "); Serial.println(D2);

  // apply compensation
  double dT   = (double)D2 - ((double)C[5] * 256.0);
  double TEMP = 2000.0 + dT * ((double)C[6] / 8388608.0);  // 2^23 = 8 388 608 :contentReference[oaicite:4]{index=4}&#8203;:contentReference[oaicite:5]{index=5}

  double OFF  = (double)C[2] * 131072.0 + ( (double)C[4] * dT ) / 64.0; 
  double SENS = (double)C[1] * 65536.0 + ( (double)C[3] * dT ) / 128.0;

  Serial.print("dT                = "); Serial.println(dT);
  Serial.print("OFF               = "); Serial.println(OFF);
  Serial.print("SENS              = "); Serial.println(SENS);

  double P    = ( ( (double)D1 * SENS ) / 2097152.0 - OFF ) / (32768.0 * 100);   // 2^21=2 097 152, 2^15=32 768

  // print human‑readable values
  Serial.print("Pressure: ");
  Serial.print(P, 2);
  Serial.print(" mbar,  Temperature: ");
  Serial.print(TEMP / 100.0, 2);
  Serial.println(" °C");

  delay(1000);
}

void resetSensor() {
  SPI.beginTransaction(ms5607SPI);
  digitalWrite(CSB_PIN, LOW);
    SPI.transfer(0x1E);    // Reset command :contentReference[oaicite:6]{index=6}&#8203;:contentReference[oaicite:7]{index=7}
  digitalWrite(CSB_PIN, HIGH);
  SPI.endTransaction();
}

void readCalibration() {
  SPI.beginTransaction(ms5607SPI);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(CSB_PIN, LOW);
      SPI.transfer(0xA0 | (i << 1));    // PROM read command, addr = i :contentReference[oaicite:8]{index=8}&#8203;:contentReference[oaicite:9]{index=9}
      uint16_t hi = SPI.transfer(0x00);
      uint16_t lo = SPI.transfer(0x00);
    digitalWrite(CSB_PIN, HIGH);
    C[i] = (hi << 8) | lo;
    Serial.print("C["); Serial.print(i); Serial.print("] = "); Serial.println(C[i]);
  }
  SPI.endTransaction();
}

uint32_t readADC(uint8_t cmd, uint16_t delayMs) {
  uint32_t val = 0;
  SPI.beginTransaction(ms5607SPI);

  // start conversion
  digitalWrite(CSB_PIN, LOW);
    SPI.transfer(cmd);
  digitalWrite(CSB_PIN, HIGH);

  // wait for conversion to complete
  delay(delayMs);

  // read back 24 bits
  digitalWrite(CSB_PIN, LOW);
    SPI.transfer(0x00);   // ADC read command
    val  = (uint32_t)SPI.transfer(0x00) << 16;
    val |= (uint32_t)SPI.transfer(0x00) << 8;
    val |= (uint32_t)SPI.transfer(0x00);
  digitalWrite(CSB_PIN, HIGH);

  SPI.endTransaction();
  return val;
}
