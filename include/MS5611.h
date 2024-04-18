#ifndef MS5611_H
#define MS5611_H

#include "MS5611_Base.h"

//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76 (0b1110110)
//  CS to GND  ==>  0x77 (0b1110111)

/*
 ******************************************************************************
 *	从PROM读取工厂的出厂数据
 *	C0	为0正常，为1说明IIC通信有问题或者硬件复位后延时不够
 *	C1  压力敏感度 SENS_T1				uint16_t
 *	C2  压力补偿  OFF_T1				uint16_t
 *	C3	温度压力灵敏度系数 TCS          uint16_t
 *	C4	温度系数的压力补偿 TCO          uint16_t
 *	C5	参考温度 T_REF                  uint16_t
 *	C6 	温度系数 TEMPSENS				uint16_t
 *	C7	CRC校验
 ******************************************************************************
 *	读取数字压力和温度数据
 *
 *	D1	数字压力值						uint32_t
 *	D2	数字温度值						uint32_t
 ******************************************************************************
 *	计算温度
 *
 *	dT		实际温度与参考温度之差	dT=D2-C5*2^8			int32_t
 *	TEMP	实际温度				TEMP=2000+dT*C6/2^23	int32_t	2007(20.07℃)
 ******************************************************************************
 *	计算温度补偿压力
 *
 *	OFF		实际温度补偿		OFF=C2*2^16+(C4*dT)/2^7		int64_t
 *	SENS	实际温度下的灵敏度	SENS=C1*2^15+(C3*dT)/2^8	int64_t
 *	P		温度补偿压力		P=(D1*SENS/2^21-OFF)/2^15	int32_t	100009(1000.09mbar=100kPa)
 ******************************************************************************
 */

#define MS5611_LIB_VERSION (F("0.0.1-alpha"))

class MS5611 {
public:
    MS5611(MS5611_Base& ms5611_xxx) : _ms5611(ms5611_xxx) {}

    bool begin() { return _ms5611.begin(); };
    bool isConnected() { return _ms5611.isConnected(); };

    bool reset() { return _ms5611.reset(); };

    int read(osr_t oversamplingRate) { return _ms5611.read(oversamplingRate); };
    inline int read() { return _ms5611.read(); };

    osr_t getOversampling() const { return _ms5611.getOversampling(); };

    float getTemperature() const { return _ms5611.getTemperature(); };

    float getPressure() const { return _ms5611.getPressure(); };

    float getHeight(h_mode mode = ONLY_PRESSURE) const { return _ms5611.getHeight(mode); };

    float getTemperatureOffset() { return _ms5611.getTemperatureOffset(); };

    float getPressureOffset() { return _ms5611.getPressureOffset(); };

    bool getCompensation() { return _ms5611.getCompensation(); };

    void setOversampling(osr_t overSamplingRate) { _ms5611.setOversampling(overSamplingRate); };

    void setPressureOffset(float offset = 0) { _ms5611.setPressureOffset(offset); };

    void setTemperatureOffset(float offset = 0) { _ms5611.setTemperatureOffset(offset); };

    void setCompensation(bool flag = true) { _ms5611.setCompensation(flag); };

    int getLastResult() const { return _ms5611.getLastResult(); };

    uint32_t getDeviceID() const { return _ms5611.getDeviceID(); };

    uint32_t lastRead() const { return _ms5611.lastRead(); };

    void list() { _ms5611.list(); };

private:
    MS5611_Base& _ms5611;
};

#endif
