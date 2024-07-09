#include <MS5611_IIC.h>

/////////////////////////////////////////////////////
//
//  PUBLIC
//

#if defined(ESP8266) || defined(ESP32)

bool MS5611_IIC::begin() {
    if ((_address < 0x76) || (_address > 0x77)) return false;

    if ((_sda < 255) && (_scl < 255)) {
        _wire->begin(_sda, _scl);
    } else {
        _wire->begin();
    }
    if (!isConnected()) return false;

    return reset();
}
#endif

bool MS5611_IIC::isConnected() {
    _wire->beginTransmission(_address);
#ifdef ARDUINO_ARCH_NRF52840
    //  needed for NANO 33 BLE
    _wire->write(0);
#endif
    return (_wire->endTransmission() == 0);
}

/////////////////////////////////////////////////////
//
//  PRIVATE
//
int MS5611_IIC::command(const uint8_t command) {
    yield();
    _wire->beginTransmission(_address);
    _wire->write(command);
    _result = _wire->endTransmission();
    return _result;
}
uint16_t MS5611_IIC::readProm(uint8_t reg) {
    //  last EEPROM register is CRC - Page 13 datasheet.
    uint8_t promCRCRegister = 7;
    if (reg > promCRCRegister) return 0;

    uint8_t offset = reg * 2;
    command(MS5611_CMD_READ_PROM + offset);
    if (_result == 0) {
        uint8_t length = 2;
        int bytes = _wire->requestFrom(_address, length);
        if (bytes >= length) {
            uint16_t value = _wire->read() * 256;
            value += _wire->read();
            return value;
        }
        return 0;
    }
    return 0;
}

uint32_t MS5611_IIC::readADC() {
    command(MS5611_CMD_READ_ADC);
    if (_result == 0) {
        uint8_t length = 3;
        int bytes = _wire->requestFrom(_address, length);
        if (bytes >= length) {
            uint32_t value = _wire->read() * 65536UL;
            value += _wire->read() * 256UL;
            value += _wire->read();
            return value;
        }
        return 0UL;
    }
    return 0UL;
}

// -- END OF FILE --
