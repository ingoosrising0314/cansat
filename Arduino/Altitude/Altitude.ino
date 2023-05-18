
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SCL 8
#define SDA 9
#define CSB 10
#define SDO 11​

Adafruit_BMP280 bmp(CSB,SDA,SDO,SCL); // I2C

void setup() {
  Serial.begin(9600);
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

​

void loop() {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");​
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1023.8)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(2000);
}
