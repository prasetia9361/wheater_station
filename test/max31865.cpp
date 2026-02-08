#include <Arduino.h>
#include <Adafruit_MAX31865.h>

// Software SPI: Pins can be changed
#define MAX31865_CS   10
#define MAX31865_MOSI 11
#define MAX31865_MISO 13
#define MAX31865_CLK  12

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

// PT100 or PT1000 settings
#define RREF      430.0    // PT100 (430 for 3wire, 4300 for PT1000)
#define RNOMINAL  100.0    // 100 for PT100, 1000 for PT1000

void setup() {
  Serial.begin(115200);
  thermo.begin(MAX31865_2WIRE);
}

void loop() {
  uint16_t rtd = thermo.readRTD();
  float temperature = thermo.temperature(RNOMINAL, RREF);

  Serial.print("RTD Value: "); Serial.println(rtd);
  Serial.print("Temperature: "); Serial.println(temperature);

  // Check for faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    thermo.clearFault();
  }
  
  delay(1000);
}
