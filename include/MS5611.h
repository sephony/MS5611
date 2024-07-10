/*
 * @author      sephony
 * @date        2024-07-09
 * @version     1.2.0
 * @brief       MS5611气压传感器驱动库
 * @details     本库基于MS5611传感器，实现了IIC和SPI两种通信方式的驱动库。
 *------------------------------------------------------------------------------
 * MS5611-01BA数据手册:
 *
 * MODE         SPI    I2C    pin     MS5611
 *                                  +--------+
 *              VCC    VCC    VCC---| o      |
 *              GND    GND    GND---| o      |
 *              SCK    SCL    SCL---| o      |
 *             MOSI    SDA    SDA---| o      |
 *               CS    CSB    CSB---| o      |
 *             MISO           SDO---| o L    |   L = led
 *              LOW   HIGH     PS---| o      |   PS = protocol select
 *                                  +--------+
 *
 *          IIC模式下:
 *              PS to VCC  ==>  I2C  (别忘了IIC总线的上拉电阻)
 *              CSB to VCC  ==>  0x76 (0b1110110)
 *              CSB to GND  ==>  0x77 (0b1110111)
 *              SDO可悬空
 *
 *          SPI模式下:
 *              PS to GND  ==>  SPI
 *
 * @note SPI模式下传感器会有积热现象
 * -----------------------------------------------------------------------------
 */
#ifndef MS5611_H
#define MS5611_H

#include <MS5611_Base.h>
#include <MS5611_IIC.h>
#include <MS5611_SPI.h>

#define MS5611_LIB_VERSION (F("0.0.1-alpha"))

class MS5611 {
public:
    MS5611(MS5611_Base& ms5611_xxx) : _ms5611(ms5611_xxx) {}

    /*
     * @brief 初始化传感器
     * @return true: 初始化成功; false: 初始化失败
     */
    bool begin() { return _ms5611.begin(); };

    /*
     * @brief 检查传感器是否成功连接
     * @return true: 连接成功; false: 连接失败
     */
    bool isConnected() { return _ms5611.isConnected(); };

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
    bool reset() { return _ms5611.reset(); };

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
    int read(osr_t oversamplingRate) { return _ms5611.read(oversamplingRate); };

    inline int read() { return _ms5611.read(); };

    /*
     * @brief 获得当前采样率
     * @return 256:OSR_ULTRA_LOW; 512:OSR_LOW; 1024:OSR_STANDARD; 2048:OSR_HIGH; 4096:OSR_ULTRA_HIGH
     */
    osr_t getOversampling() const { return _ms5611.getOversampling(); };

    /*
     * @brief 获得当前温度
     * @return 温度（℃）
     */
    double getTemperature() const { return _ms5611.getTemperature(); };

    /*
     * @brief 获得当前气压
     * @return 气压（kPa）
     */
    double getPressure() const { return _ms5611.getPressure(); };

    /*
     * @brief 获得当前海拔高度
     * @param mode 高度计算模式
     * @return 高度（m）
     * @note 默认为仅使用气压计算高度(`ONLY_PRESSURE`)，若需要使用气压温度混合计算高度，请设置`mode`为`MIXED`
     */
    double getHeight(h_mode mode = ONLY_PRESSURE) { return _ms5611.getHeight(mode); };

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
    void init(uint32_t delay_time = 30, uint8_t n = 100, h_mode mode = ONLY_PRESSURE) { return _ms5611.init(delay_time, n, mode); };

    /*
     * @brief 获得初始平均温度、气压与海拔高度
     * @param data 数据类型（`T0`: 初始温度; `P0`: 初始气压; `H0`: 初始海拔高度）
     * @return 初始值
     */
    double getInit(const std::string& data) { return _ms5611.getInit(data); }

    /*
     * @brief 获得相对海拔高度（滤波后）
     * @param mode 高度计算模式
     * @return 高度（m）
     * @note 默认为仅使用气压计算高度(`ONLY_PRESSURE`)，若需要使用气压温度混合计算高度，请设置`mode`为`MIXED`
     */
    double getRelativeHeight(h_mode mode = ONLY_PRESSURE) { return _ms5611.getRelativeHeight(mode); };

    /*
     * @brief 获得当前温度偏移
     * @return 温度偏移（℃）
     */
    double getTemperatureOffset() { return _ms5611.getTemperatureOffset(); };

    /*
     * @brief 获得当前气压偏移
     * @return 气压偏移（kPa）
     */
    double getPressureOffset() { return _ms5611.getPressureOffset(); };

    /*
     * @brief 获得当前补偿状态（是否开启）
     * @return 0: 未开启; 1: 开启
     */
    bool getCompensation() { return _ms5611.getCompensation(); };

    /*
     * @brief 设置采样率
     * @param overSamplingRate 采样率
     * @note 可选参数
     *       `OSR_ULTRA_LOW`; `OSR_LOW`; `OSR_STANDARD`; `OSR_HIGH`; `OSR_ULTRA_HIGH`
     */
    void setOversampling(osr_t overSamplingRate) { _ms5611.setOversampling(overSamplingRate); };

    /*
     * @brief 设置温度偏移
     * @param 温度偏移（℃）
     */
    void setTemperatureOffset(double offset = 0) { _ms5611.setTemperatureOffset(offset); };

    /*
     * @brief 设置气压偏移
     * @param 气压偏移（kPa）
     */
    void setPressureOffset(double offset = 0) { _ms5611.setPressureOffset(offset); };

    /*
     * @brief 设置补偿状态(默认开启)
     * @param flag true: 开启; false: 关闭
     */
    void setCompensation(bool flag = true) { _ms5611.setCompensation(flag); };

    /*
     * @brief 获得上一次读取的结果（仅IIC通信时返回有效值）
     * @return 0: 成功
     *         1: 数据太长超过发送缓存区
     *         2: 在传输地址时没有收到应答（NACK）
     *         3: 在传输数据时没有收到应答（NACK）
     *         4: 其他错误
     *         5: 超时
     */
    int getLastResult() const { return _ms5611.getLastResult(); };

    /*
     * @brief 获取设备ID
     * @return 设备ID
     * @details _deviceID is a SHIFT XOR merge of 7 PROM registers, reasonable unique
     */
    uint32_t getDeviceID() const { return _ms5611.getDeviceID(); };

    /*
     * @brief 获得最后一次读取的时间
     * @return 时间（ms）
     */
    uint32_t lastRead() const { return _ms5611.lastRead(); };

    /*
     * @brief 列出传感器出厂校准值（C0~C7）及读取的ADC转换值(D1、D2)
     */
    void list() { _ms5611.list(); };

    /*
     * @brief 列出传感器出厂校准值（C0~C7）及读取的ADC转换值(D1、D2)
     */
    void debug() { _ms5611.debug(); };

private:
    MS5611_Base& _ms5611;
};

#endif  // MS5611_H
