#ifndef MS5611_SPI_H
#define MS5611_SPI_H

#include <Arduino.h>
#include <MS5611_Base.h>
#include <SPI.h>

class MS5611_SPI : public MS5611_Base {
public:
    explicit MS5611_SPI(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
        : _sck(sck), _miso(miso), _mosi(mosi), _ss(ss), _hwSPI(true), _SPIspeed(1000000), _spi_settings(1000000, MSBFIRST, SPI_MODE0) {}

    MS5611_SPI(int8_t ss) : MS5611_SPI(255, 255, 255, ss) {}

    bool begin() override;
    bool isConnected() override;

    //       speed in Hz
    void setSPIspeed(uint32_t speed);
    uint32_t getSPIspeed();

    //  debugging
    bool usesHWSPI();
    bool end();

private:
    int command(const uint8_t command) override;
    uint16_t readProm(uint8_t reg) override;
    uint32_t readADC() override;

    uint8_t _address;

    //  SPI
    uint8_t _sck;
    uint8_t _miso;
    uint8_t _mosi;
    uint8_t _ss;
    bool _hwSPI = true;
    uint32_t _SPIspeed = 1000000;
    uint8_t swSPI_transfer(uint8_t value);

    SPIClass *mySPI;
    SPISettings _spi_settings;
    bool _useHSPI = false;
};

#endif  // MS5611_SPI_H
