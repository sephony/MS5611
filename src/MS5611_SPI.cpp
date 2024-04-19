//
//    FILE: MS5611.cpp
//  AUTHOR: Qiao Dong
// VERSION: 0.0.1
// PURPOSE: MS5611 (SPI) Temperature & Pressure library for ESP32 with Arduino framework
//     URL: https://github.com/QiaoDong1216/MS5611
//
//  HISTORY: see changelog.md

#include "MS5611_SPI.h"

/*********************************PRIVATE**************************************/

bool MS5611_SPI::begin() {
    //  print experimental message.
    //  Serial.println(MS5611_SPI_LIB_VERSION);
    //  SPI
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    if (_sck == 255 && _miso == 255 && _mosi == 255) {
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        _sck = 12;
        _miso = 13;
        _mosi = 11;
#endif
    }
    Serial.print("sck: ");
    Serial.println(_sck);
    Serial.print("miso: ");
    Serial.println(_miso);
    Serial.print("mosi: ");
    Serial.println(_mosi);
    if (_hwSPI) {
        mySPI = new SPIClass(FSPI);
        mySPI->begin(_sck, _miso, _mosi, _ss);
        delay(1);
    } else {
        //  Serial.println("SW_SPI");
        pinMode(_sck, OUTPUT);
        pinMode(_miso, INPUT);
        pinMode(_mosi, OUTPUT);
        pinMode(_ss, OUTPUT);
        digitalWrite(_sck, HIGH);
        digitalWrite(_mosi, LOW);
        digitalWrite(_ss, HIGH);
    }
    return reset();
}

bool MS5611_SPI::isConnected() {
    int rv = read();
    return (rv == MS5611_READ_OK);
}

void MS5611_SPI::setSPIspeed(uint32_t speed) {
    _SPIspeed = speed;
    _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
};

uint32_t MS5611_SPI::getSPIspeed() {
    return _SPIspeed;
};

bool MS5611_SPI::usesHWSPI() {
    return _hwSPI;
};

bool MS5611_SPI::end() {
    if (_hwSPI) {
        SPI.end();
    }
    return true;
}

/*********************************PRIVATE**************************************/
int MS5611_SPI::command(const uint8_t command) {
    yield();
    digitalWrite(_ss, LOW);
    if (_hwSPI) {
        mySPI->beginTransaction(_spi_settings);
        mySPI->transfer(command);
        mySPI->endTransaction();
    } else  //  Software SPI
    {
        swSPI_transfer(command);
    }
    digitalWrite(_ss, HIGH);
    return 0;
}

uint16_t MS5611_SPI::readProm(uint8_t reg) {
    //  last EEPROM register is CRC - Page 13 datasheet.
    uint8_t promCRCRegister = 7;
    if (reg > promCRCRegister) return 0;

    uint16_t value = 0;
    digitalWrite(_ss, LOW);
    if (_hwSPI) {
        mySPI->beginTransaction(_spi_settings);
        mySPI->transfer(MS5611_CMD_READ_PROM + reg * 2);
        value += mySPI->transfer(0x00);
        value <<= 8;
        value += mySPI->transfer(0x00);
        mySPI->endTransaction();
    } else  //  Software SPI
    {
        swSPI_transfer(MS5611_CMD_READ_PROM + reg * 2);
        value += swSPI_transfer(0x00);
        value <<= 8;
        value += swSPI_transfer(0x00);
    }
    digitalWrite(_ss, HIGH);
    return value;
}

uint32_t MS5611_SPI::readADC() {
    //  command(MS5611_CMD_READ_ADC);

    uint32_t value = 0;

    digitalWrite(_ss, LOW);
    if (_hwSPI) {
        mySPI->beginTransaction(_spi_settings);
        mySPI->transfer(0x00);
        value += mySPI->transfer(0x00);
        value <<= 8;
        value += mySPI->transfer(0x00);
        value <<= 8;
        value += mySPI->transfer(0x00);
        mySPI->endTransaction();
    } else  //  Software SPI
    {
        swSPI_transfer(0x00);
        value += swSPI_transfer(0x00);
        value <<= 8;
        value += swSPI_transfer(0x00);
        value <<= 8;
        value += swSPI_transfer(0x00);
    }
    digitalWrite(_ss, HIGH);
    //  Serial.println(value, HEX);
    return value;
}

//  simple one mode version
uint8_t MS5611_SPI::swSPI_transfer(uint8_t val) {
    uint8_t clk = _sck;
    uint8_t dao = _mosi;
    uint8_t dai = _miso;
    uint8_t value = 0;
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        digitalWrite(dao, (val & mask));
        digitalWrite(clk, HIGH);
        value <<= 1;
        if (digitalRead(dai) != 0) value += 1;
        digitalWrite(clk, LOW);
    }
    digitalWrite(dao, LOW);
    //  Serial.print(" # ");
    //  Serial.println(value, HEX);
    return value;
}

// -- END OF FILE --
