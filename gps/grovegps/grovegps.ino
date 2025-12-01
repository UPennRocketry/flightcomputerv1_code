#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
SoftwareSerial ss(4, 3); // GPS TX → pin 4, RX ← pin 3 for arduino uno


void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  // Enable RMC, GGA, GSA, GSV
  ss.print("$PGKC242,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*3F\r\n");

  Serial.println("TinyGPS++ GNSS reader");
}

void loop() {
  while (ss.available()) {
    char c = ss.read();
    Serial.write(c);
    Serial.println();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    Serial.print("LAT: "); Serial.println(gps.location.lat(), 6);
    Serial.print("LON: "); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT="); Serial.println(gps.altitude.meters(), 6);
    Serial.print("SATS: "); Serial.println(gps.satellites.value());
    Serial.print("HDOP: "); Serial.println(gps.hdop.hdop());
    Serial.println();
  } else {
    Serial.println("Waiting for fix...");
    Serial.print("SATS: ");
    Serial.println(gps.satellites.value());
  }

  delay(1000);
}
