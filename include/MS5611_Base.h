#ifndef MS5611_BASE_H
#define MS5611_BASE_H

#include <Arduino.h>
#include <Filter.h>

#define MS5611_READ_OK 0
#define MS5611_ERROR_2 2  //  low level I2C error
#define MS5611_NOT_READ -999

#define MS5611_CMD_READ_ADC 0x00
#define MS5611_CMD_RESET 0x1E       // 0x 0001 1110
#define MS5611_CMD_CONVERT_D1 0x40  // 0x 0100 0000	(OSR:Over Sampling Ratio)
#define MS5611_CMD_CONVERT_D2 0x50  // 0x 0101 0000
#define MS5611_CMD_READ_PROM 0xA0   // 0x 1010 xxx0 (0xA0 ~ 0xAE)
#define MS5611_PROM_CRC 0xAE        // 0x 1010 1110

enum osr_t : int {
    OSR_ULTRA_LOW = 256,   //  1 millis    Default = backwards compatible
    OSR_LOW = 512,         //  2 millis
    OSR_STANDARD = 1024,   //  3 millis
    OSR_HIGH = 2048,       //  5 millis
    OSR_ULTRA_HIGH = 4096  // 10 millis
};

enum h_mode : int {
    ONLY_PRESSURE = 0,  // 仅气压计算高度
    MIXED = 1,          // 气压温度混合计算高度
};

class MS5611_Base {
public:
    virtual bool begin() = 0;
    virtual bool isConnected() = 0;

    //       reset command + get constants
    //       mathMode = 0 (default), 1 = factor 2 fix.
    //       returns false if ROM constants == 0;
    bool reset();
    //  the actual reading of the sensor;
    //  returns MS5611_READ_OK upon success
    int read(osr_t oversamplingRate);
    //  wrapper, uses the preset oversampling rate.
    inline int read() { return read(_overSamplingRate); };

    //  oversampling rate is in osr_t
    osr_t getOversampling() const { return _overSamplingRate; };

    //  temperature is in ²C
    float getTemperature() const { return _temperature; };

    //  pressure is in mBar
    float getPressure() const { return _pressure; };

    float getHeight(h_mode mode = ONLY_PRESSURE);
    float getInitHeight();
    float getRelativeHeight(h_mode mode = ONLY_PRESSURE);

    float getTemperatureOffset() { return _temperatureOffset; };
    float getPressureOffset() { return _pressureOffset; };

    bool getCompensation() { return _compensation; };
    //  sets oversampling to a value between 8 and 12
    void setOversampling(osr_t overSamplingRate) { _overSamplingRate = overSamplingRate; };

    //  OFFSET - 0.3.6
    void setPressureOffset(float offset = 0) { _pressureOffset = offset; };

    void setTemperatureOffset(float offset = 0) { _temperatureOffset = offset; };

    void setCompensation(bool flag = true) { _compensation = flag; };

    //  to check for failure
    int getLastResult() const { return _result; };

    //  _deviceID is a SHIFT XOR merge of 7 PROM registers, reasonable unique
    uint32_t getDeviceID() const { return _deviceID; };

    //  last time in millis() when the sensor has been read.
    uint32_t lastRead() const { return _lastRead; };

    void list();

protected:
    virtual int command(const uint8_t command) = 0;
    virtual uint16_t readProm(uint8_t reg) = 0;
    virtual uint32_t readADC() = 0;

    void convert(const uint8_t addr, osr_t overSamplingRate);

    uint16_t C[8];
    uint32_t D1;
    uint32_t D2;
    int32_t dT;
    int32_t TEMP;
    int64_t OFF;
    int64_t SENS;
    int32_t P;

    bool _compensation = true;

    int _result = 0;  // 在IIC通信中，返回0表示成功；SPI通信中，该变量一直为0，无意义

private:
    osr_t _overSamplingRate = OSR_STANDARD;  // 采样率
    float _temperature = MS5611_NOT_READ;    // 温度
    float _pressure = MS5611_NOT_READ;       // 气压
    float _pressureOffset = 0;               // 气压偏移
    float _temperatureOffset = 0;            // 温度偏移

    float _height = MS5611_NOT_READ;         // 海拔高度
    float _H0 = 0;                           // 初始海拔高度
    float _height_filter = MS5611_NOT_READ;  // 滤波后的相对高度
    Filter AltitudeLPF_50;                   // 50Hz Butterworth LPF

    uint32_t _lastRead = MS5611_NOT_READ;
    uint32_t _deviceID = MS5611_NOT_READ;
};

#endif  // MS5611_BASE_H
