#include <Arduino.h>
#include <U8g2lib.h>
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

uint32_t start, stop, start_ms5611, stop_ms5611;
double relative_height = 0;

// U8x8 character mode
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/16, /* data=*/17, /* reset=*/U8X8_PIN_NONE);

void setup() {
    Serial.begin(115200);

    // u8x8.begin();

    pinMode(LED_BUILTIN, OUTPUT);
#ifdef MS5611_USE_IIC
    pinMode(MS5611_CS, OUTPUT);
    digitalWrite(MS5611_CS, LOW);
#endif
    u8g2.begin();

    if (ms5611.begin()) {
        Serial.print("MS5611 found! ID: ");
        Serial.println(ms5611.getDeviceID(), HEX);
    } else {
        Serial.println("MS5611 not found. halt.");
        while (1) {
            delay(1000);
        }
    }
    ms5611.init();  // 获取初始平均高度
}

void loop() {
    start_ms5611 = millis();
    ms5611.read();
    stop_ms5611 = millis();
    relative_height = ms5611.getRelativeHeight();
    Serial.print("Temperature: ");
    Serial.print(ms5611.getTemperature(), 2);
    Serial.print(" C, Pressure: ");
    Serial.print(ms5611.getPressure(), 2);
    Serial.print(" mBar, Height: ");
    Serial.print(relative_height, 2);
    Serial.print(" m,\t Duration: ");
    Serial.print(stop_ms5611 - start_ms5611);
    Serial.println(" ms");

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Temperature:");
    u8g2.drawStr(0, 20, "Pressure:");
    u8g2.drawStr(0, 30, "Height:");
    u8g2.drawStr(0, 40, "Duration:");
    u8g2.drawStr(0, 50, "T0:");
    u8g2.drawStr(0, 60, "P0:");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(75, 10);
    u8g2.print(ms5611.getTemperature(), 2);
    u8g2.setCursor(75, 20);
    u8g2.print(ms5611.getPressure(), 2);
    u8g2.setCursor(75, 30);
    u8g2.print(relative_height, 7);
    u8g2.setCursor(75, 40);
    stop = millis();
    u8g2.print(stop - start);
    start = millis();
    u8g2.setCursor(75, 50);
    u8g2.print(ms5611.getInit("T0"));
    u8g2.setCursor(75, 60);
    u8g2.print(ms5611.getInit("P0"));
    u8g2.sendBuffer();
}
// -- END OF FILE --
