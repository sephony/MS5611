#include <Arduino.h>
#include <Wire.h>

#include "MS5611.h"

#define MS5611_USE_IIC  // 采用IIC

// IIC scl->12, sda->11
constexpr uint8_t pin_scl = 12;
constexpr uint8_t pin_sda = 11;

// SPI sck->12 miso->13 mosi->11 cs->5
constexpr uint8_t MS5611_CS = 5;

#ifdef MS5611_USE_IIC
MS5611_IIC ms5611_iic(pin_sda, pin_scl);  // ESP32 IIC
MS5611 ms5611(ms5611_iic);
#else
MS5611_SPI ms5611_spi(MS5611_CS);  // ESP32 SPI
MS5611 ms5611(ms5611_spi);
#endif

void setup() {
    Serial.begin(115200);

#ifdef MS5611_USE_IIC
    pinMode(MS5611_CS, OUTPUT);
    digitalWrite(MS5611_CS, LOW);
#endif

    if (ms5611.begin()) {
        Serial.print("MS5611 found! ID: ");
        Serial.println(ms5611.getDeviceID(), HEX);
    } else {
        Serial.println("MS5611 not found.");
        while (1) {
            delay(1000);
        }
    }
    ms5611.init();  // 获取初始平均高度
}

void loop() {
    ms5611.read();
    Serial.print("Temperature: ");
    Serial.print(ms5611.getTemperature(), 2);
    Serial.print(" C, Pressure: ");
    Serial.print(ms5611.getPressure(), 2);
    Serial.print(" mBar, Height: ");
    Serial.print(ms5611.getRelativeHeight(), 2);
    Serial.print(" m,\t Duration: ");
    Serial.print(ms5611.getTimeBetweenRead());
    Serial.println(" ms");
}
