#ifndef MS5611_BASE_H
#define MS5611_BASE_H

#include <Arduino.h>
#include <Filter.h>
#include <string.h>

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

#define MS5611_READ_OK 0
#define MS5611_ERROR_2 2  // todo: low level I2C error
#define MS5611_NOT_READ -999

#define MS5611_CMD_READ_ADC 0x00    // ADC读取命令
#define MS5611_CMD_RESET 0x1E       // 0x 0001 1110
#define MS5611_CMD_CONVERT_D1 0x40  // 0x 0100 0000	(OSR:Over Sampling Ratio)
#define MS5611_CMD_CONVERT_D2 0x50  // 0x 0101 0000
#define MS5611_CMD_READ_PROM 0xA0   // 0x 1010 xxx0 (0xA0 ~ 0xAE)
#define MS5611_PROM_CRC 0xAE        // 0x 1010 1110

// 采样率（ADC转换时间）
enum osr_t : int {
    OSR_ULTRA_LOW = 256,   //  1 ms
    OSR_LOW = 512,         //  2 ms
    OSR_STANDARD = 1024,   //  3 ms
    OSR_HIGH = 2048,       //  5 ms
    OSR_ULTRA_HIGH = 4096  // 10 ms
};

// 高度计算模式
enum h_mode : int {
    ONLY_PRESSURE = 0,  // 仅气压计算高度
    MIXED = 1,          // 气压温度混合计算高度
};

class MS5611_Base {
public:
    /*
     * @brief 初始化传感器
     * @return bool
     */
    virtual bool begin() = 0;

    /*
     * @brief 检查传感器是否成功连接
     * @return bool
     */
    virtual bool isConnected() = 0;

    /*
     * @brief 重置MS5611传感器，并读取出厂PROM数据。
     *
     * @details 这个函数发送一个复位序列给MS5611传感器，以确保在上电后校准PROM正确加载到内部寄存器。
     *       它也可以在传感器处于未知状态时用来重置设备。
     *
     * @note 在读取PROM数据之前必须reset芯片，
     *       reset芯片后，需要要延时一段时间才能读取出厂校准值（函数内部已添加延时）。
     *
     * @return 如果复位成功返回true，否则返回false。
     */
    bool reset();

    /*
     * @brief 读取传感器数据
     *
     * @param oversamplingRate 采样率
     *
     * @return 0: 成功
     *         1: 数据太长超过发送缓存区
     *         2: 在传输地址时没有收到应答（NACK）
     *         3: 在传输数据时没有收到应答（NACK）
     *         4: 其他错误
     *         5: 超时
     *
     * @note 只有在IIC通信时才可能返回错误码，SPI通信时不会返回错误码。
     *
     * @details 若出现错误（返回非0值），由IIC通信函数endTransmission()实际报告:
     *          参见https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
     */
    int read(osr_t oversamplingRate);

    //  wrapper, uses the preset oversampling rate.
    inline int read() { return read(_overSamplingRate); };

    /*
     * @brief 获得当前采样率
     * @return 256:OSR_ULTRA_LOW; 512:OSR_LOW; 1024:OSR_STANDARD; 2048:OSR_HIGH; 4096:OSR_ULTRA_HIGH
     */
    osr_t getOversampling() const { return _overSamplingRate; };

    /*
     * @brief 获得当前温度
     * @return 温度（℃）
     */
    double getTemperature() const { return _temperature; };

    /*
     * @brief 获得当前气压
     * @return 气压（kPa）
     */
    double getPressure() const { return _pressure; };

    /*
     * @brief 获得当前海拔高度
     * @param mode 高度计算模式
     * @return 高度（m）
     * @note 默认为仅使用气压计算高度(`ONLY_PRESSURE`)，若需要使用气压温度混合计算高度，请设置`mode`为`MIXED`
     */
    double getHeight(h_mode mode = ONLY_PRESSURE);

    /*
     * @brief 计算初始平均温度、气压与海拔高度
     *
     * @param delay_time 采样间隔时间（ms）
     * @param n 采样次数
     * @param mode 高度计算模式
     *
     * @note 默认为仅使用气压计算高度(`ONLY_PRESSURE`)，若需要使用气压温度混合计算高度，请设置`mode`为`MIXED`
     * @note 采样间隔非常重要，不同的采样间隔会影响最终的平均值！loop程序中实时采样的间隔尽量与此处设置的间隔一致。
     *       或者添加偏移量，使得采样间隔的误差对最终结果的影响降到最低。
     */
    void init(uint32_t delay_time = 30, uint8_t n = 100, h_mode mode = ONLY_PRESSURE);

    /*
     * @brief 获得初始平均温度、气压与海拔高度
     * @param data 数据类型（`T0`: 初始温度; `P0`: 初始气压; `H0`: 初始海拔高度）
     * @return 初始值
     */
    double getInit(const std::string& data);

    /*
     * @brief 获得相对海拔高度（滤波后）
     * @param mode 高度计算模式
     * @return 高度（m）
     * @note 默认为仅使用气压计算高度(`ONLY_PRESSURE`)，若需要使用气压温度混合计算高度，请设置`mode`为`MIXED`
     */
    double getRelativeHeight(h_mode mode = ONLY_PRESSURE);

    /*
     * @brief 获得当前温度偏移
     * @return 温度偏移（℃）
     */
    double getTemperatureOffset() { return _temperatureOffset; };

    /*
     * @brief 获得当前气压偏移
     * @return 气压偏移（kPa）
     */
    double getPressureOffset() { return _pressureOffset; };

    /*
     * @brief 获得当前补偿状态（是否开启）
     * @return 0: 未开启; 1: 开启
     */
    bool getCompensation() { return _compensation; };

    /*
     * @brief 设置采样率
     * @param overSamplingRate 采样率
     * @note 可选参数
     *       `OSR_ULTRA_LOW`; `OSR_LOW`; `OSR_STANDARD`; `OSR_HIGH`; `OSR_ULTRA_HIGH`
     */
    void setOversampling(osr_t overSamplingRate) { _overSamplingRate = overSamplingRate; };

    /*
     * @brief 设置温度偏移
     * @param 温度偏移（℃）
     */
    void setPressureOffset(double offset = 0) { _pressureOffset = offset; };

    /*
     * @brief 设置气压偏移
     * @param 气压偏移（kPa）
     */
    void setTemperatureOffset(double offset = 0) { _temperatureOffset = offset; };

    /*
     * @brief 设置补偿状态(默认开启)
     * @param flag true: 开启; false: 关闭
     */
    void setCompensation(bool flag = true) { _compensation = flag; };

    /*
     * @brief 获得上一次读取的结果（仅IIC通信时返回有效值）
     * @return 0: 成功
     *         1: 数据太长超过发送缓存区
     *         2: 在传输地址时没有收到应答（NACK）
     *         3: 在传输数据时没有收到应答（NACK）
     *         4: 其他错误
     *         5: 超时
     */
    int getLastResult() const { return _result; };

    /*
     * @brief 获取设备ID
     * @return 设备ID
     * @details _deviceID is a SHIFT XOR merge of 7 PROM registers, reasonable unique
     */
    uint32_t getDeviceID() const { return _deviceID; };

    /*
     * @brief 获得最后一次读取的时间
     * @return 时间（ms）
     */
    uint32_t lastRead() const { return _lastRead; };

    /*
     * @brief 列出传感器出厂校准值（C0~C7）及读取的ADC转换值(D1、D2)
     */
    void list();

    /*
     * @brief 列出传感器出厂校准值（C0~C7）及读取的ADC转换值(D1、D2)
     */
    void debug();

protected:
    virtual int command(const uint8_t command) = 0;
    virtual uint16_t readProm(uint8_t reg) = 0;
    virtual uint32_t readADC() = 0;

    void convert(const uint8_t addr, osr_t overSamplingRate);

    uint16_t C[8];  // 出厂校准值
    uint32_t D1;    // 数字压力值
    uint32_t D2;    // 数字温度值
    int32_t dT;     // 实际温度与参考温度之差
    int32_t TEMP;   // 实际温度
    int64_t OFF;    // 实际温度补偿
    int64_t SENS;   // 实际温度下的灵敏度
    int32_t P;      // 温度补偿压力

    int _result = 0;  // 上一次读取的结果（在IIC通信中，返回0表示成功；SPI通信中，该变量一直为0，无意义）

private:
    osr_t _overSamplingRate = OSR_STANDARD;  // 采样率
    double _temperature = MS5611_NOT_READ;   // 温度
    double _pressure = MS5611_NOT_READ;      // 气压
    double _height = MS5611_NOT_READ;        // 海拔高度

    bool _compensation = true;      // 补偿状态
    double _pressureOffset = 0;     // 气压偏移
    double _temperatureOffset = 0;  // 温度偏移

    double _T0 = 0;                           // 初始温度
    double _P0 = 0;                           // 初始气压
    double _H0 = 0;                           // 初始海拔高度
    double _height_filter = MS5611_NOT_READ;  // 滤波后的相对高度
    Filter H_filter;                          // 高度滤波器
    Filter P_filter;                          // 气压滤波器

    uint32_t _lastRead = MS5611_NOT_READ;  // 最后一次读取的时间
    uint32_t _deviceID = MS5611_NOT_READ;  // 设备ID
};

#endif  // MS5611_BASE_H
