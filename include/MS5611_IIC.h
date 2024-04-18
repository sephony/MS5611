#ifndef MS5611_IIC_H
#define MS5611_IIC_H

#include "Arduino.h"
#include "MS5611_Base.h"
#include "Wire.h"

#define MS5611_CSB LOW

#if MS5611_CSB == LOW
#define MS5611_IIC_ADDR 0x77  // 0b0111 0111
#else
#define MS5611_IIC_ADDR 0x76  // 0b0111 0110
#endif

class MS5611_IIC : public MS5611_Base {
public:
    // constructor
    explicit MS5611_IIC(uint8_t sda, uint8_t scl, uint8_t deviceAddress = MS5611_IIC_ADDR, TwoWire *wire = &Wire)
        : _sda(sda), _scl(scl), _address(deviceAddress), _wire(wire){};

    bool begin();
    bool isConnected();

private:
    int command(const uint8_t command) override;
    uint16_t readProm(uint8_t reg) override;
    uint32_t readADC() override;

    uint8_t _address;
    uint8_t _sda;
    uint8_t _scl;

    TwoWire *_wire;
};

#endif  // MS5611_IIC_H
