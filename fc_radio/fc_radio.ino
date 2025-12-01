#include <SPI.h>
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>

// —————— Pin definitions ——————
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

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB-Serial */ }

  // Bring radio out of reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Init the SPI1 bus (Teensy core maps SPI1 to pins 1,26,27)
  SPI1.begin();  

  // Initialize the RF95 module
  if (!rf95.init()) {
    Serial.println("RFM95 init failed");
    while (1);
  }
  Serial.println("RFM95 init OK");

  // Set frequency to 915 MHz
  if (!rf95.setFrequency(915.0)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.println("Frequency set to 915 MHz");

  // Maximum power, using the PA_BOOST pin
  rf95.setTxPower(23, false);
}

void loop() {
  const char msg[] = "Hello from Teensy!";

  Serial.print("Sending: ");
  Serial.println(msg);

  // Send and wait for completion
  rf95.send((uint8_t*)msg, sizeof(msg)-1);
  rf95.waitPacketSent();

  Serial.println("Message sent!");

  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }
  delay(5000);
}
