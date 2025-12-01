#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS myGNSS;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port

  Wire.begin();

  if (!myGNSS.begin()) {
    Serial.println("GNSS not detected. Check wiring.");
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);         // Use UBX protocol only
  myGNSS.setNavigationFrequency(1);          // 1 Hz updates
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save I/O settings

  Serial.println("GNSS initialized.");
}

void loop() {
  if (myGNSS.getPVT()) {
    float lat = myGNSS.getLatitude() / 10000000.0;
    float lon = myGNSS.getLongitude() / 10000000.0;
    float alt = myGNSS.getAltitudeMSL() / 1000.0; // convert mm to meters
    uint8_t fix = myGNSS.getFixType();
    uint8_t sats = myGNSS.getSIV();

    Serial.print("Fix: "); Serial.print(fix);
    Serial.print(" | Sats: "); Serial.print(sats);
    Serial.print(" | Lat: "); Serial.print(lat, 6);
    Serial.print(" | Lon: "); Serial.print(lon, 6);
    Serial.print(" | Alt: "); Serial.print(alt); Serial.println(" m");
  }

  delay(1000); // 5 Hz
}
