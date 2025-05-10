
#include <Wire.h>
#include <Arduino.h>

// If your Feather uses different pins, change these
static const int I2C_SDA = 22;
static const int I2C_SCL = 20;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println();
  Serial.println("I²C Scanner Starting");

  // Initialize I²C as master on the chosen pins
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I²C devices...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("  Unknown error at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("  No I²C devices found");
  else
    Serial.print("  Scan complete, found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");

  delay(1000);  // wait 5 s before next scan
}
