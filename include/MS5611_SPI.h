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

    /*
     * @brief 初始化传感器
     * @return bool
     */
    bool begin() override;

    /*
     * @brief 检查传感器是否成功连接
     * @return bool
     */
    bool isConnected() override;

    //       speed in Hz
    void setSPIspeed(uint32_t speed);
    uint32_t getSPIspeed();

    //  debugging
    bool usesHWSPI();
    bool end();

private:
    /*
     * @brief 向MS5611从机发送命令
     * @param command 命令
     */
    int command(const uint8_t command) override;

    /*
     * @brief 读取出厂PROM数据
     * @param reg 0~7（函数内部转化位寄存器地址）
     * @return 16bit 校准值
     */
    uint16_t readProm(uint8_t reg) override;

    /*
     * @brief 读取ADC转换结果
     * @return 24bit 气压/温度值
     */
    uint32_t readADC() override;

    //  SPI
    uint8_t _sck;   //  SCK
    uint8_t _miso;  //  MISO
    uint8_t _mosi;  //  MOSI
    uint8_t _ss;    //  SS
    bool _hwSPI = true;
    uint32_t _SPIspeed = 1000000;
    uint8_t swSPI_transfer(uint8_t value);

    SPIClass *mySPI;
    SPISettings _spi_settings;
    bool _useHSPI = false;
};

#endif  // MS5611_SPI_H
