#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25) // Local sea level pressure for altitude calculation

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  // Optional: Add while(!Serial) if you want to wait for the Serial Monitor to open
  delay(1000); 
  Serial.println(F("BME280 Sensor Test"));

  // Initialize I2C communication (not always necessary as framework does it, but good practice)
  Wire.begin(); 
  
  unsigned status = bme.begin(0x76); // Use 0x76 if SDO is connected to GND, otherwise default is 0x77
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Ready to read BME280 data --");
}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F); // Convert Pa to hPa
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  // Approximate altitude calculation requires local sea level pressure (hPa)
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" meters");

  Serial.println();
  delay(2000); // Wait for 2 seconds before the next reading
}
