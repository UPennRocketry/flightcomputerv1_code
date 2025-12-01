#include <SPI.h>

// ——— Pin & Register Definitions ———
const uint8_t CS_PIN      = 38;   // SPI chip‑select
const uint8_t WHO_AM_I    = 0x4F; // WHO_AM_I register (read‑only)
const uint8_t REG_CFG_A   = 0x60; // Configuration register A
const uint8_t REG_CFG_C   = 0x62; // Configuration register C
const uint8_t REG_STATUS  = 0x67; // STATUS_REG
const uint8_t OUTX_L      = 0x68; // OUTX_L through OUTZ_H (0x68–0x6D)

// ——— SPI Helpers ———
void writeReg(uint8_t reg, uint8_t val) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);   // MSb=0 → write
  SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t readReg(uint8_t reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);   // MSb=1 → read
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // — SPI init at up to 10 MHz, MODE0 —
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(10UL*1000UL*1000UL, MSBFIRST, SPI_MODE0));

  // 1) Enable 4‑wire SPI, block‑data‑update, disable I²C → 0x34
  writeReg(REG_CFG_C, 0x34);

  // 2) Start 100 Hz, continuous high‑res mode → 0x8C
  writeReg(REG_CFG_A, 0x8C);

  // 3) Verify comms
  uint8_t id = readReg(WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(id, HEX);
  if (id != 0x40) {
    Serial.println("ERROR: bad WHO_AM_I! Check wiring/CS logic.");
    while (1) delay(500);
  }
}

void loop() {
  // 1) Poll STATUS_REG until ZYXDA (bit 3) = 1
  while (!(readReg(REG_STATUS) & 0x08)) {
    // waiting for new data…
  }

  // 2) Read 6 bytes: XL, XH, YL, YH, ZL, ZH
  uint8_t xl = readReg(OUTX_L);
  uint8_t xh = readReg(OUTX_L + 1);
  uint8_t yl = readReg(OUTX_L + 2);
  uint8_t yh = readReg(OUTX_L + 3);
  uint8_t zl = readReg(OUTX_L + 4);
  uint8_t zh = readReg(OUTX_L + 5);

  // 3) Combine into signed 16‑bit
  int16_t x = (int16_t)(xh << 8 | xl);
  int16_t y = (int16_t)(yh << 8 | yl);
  int16_t z = (int16_t)(zh << 8 | zl);

  // 4) Print
  Serial.print("X = "); Serial.print(x);
  Serial.print("   Y = "); Serial.print(y);
  Serial.print("   Z = "); Serial.println(z);

  delay(100);
}
