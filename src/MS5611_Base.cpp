#include <MS5611_Base.h>

bool MS5611_Base::reset() {
    command(MS5611_CMD_RESET);
    uint32_t start = micros();

    //  while loop prevents blocking RTOS
    while (micros() - start < 2800) {
        yield();
        delayMicroseconds(10);
    }

    // 从EEPROM中读取出厂校准值
    for (uint8_t reg = 0; reg < 8; ++reg) {
        uint16_t tmp = readProm(reg);
        C[reg] = tmp;
        //  _deviceID is a SHIFT XOR merge of 7 PROM registers, reasonable unique
        _deviceID <<= 4;
        _deviceID ^= tmp;
    }
    return checkCRC();
}

int MS5611_Base::read(osr_t overSamplingRate) {
    // 读取数字压力值
    convert(MS5611_CMD_CONVERT_D1, overSamplingRate);
    if (_result) return _result;
    D1 = readADC();
    if (_result) return _result;

    // 读取数字温度值
    convert(MS5611_CMD_CONVERT_D2, overSamplingRate);
    if (_result) return _result;
    D2 = readADC();
    if (_result) return _result;

    dT = D2 - (((uint32_t)C[5]) << 8);
    TEMP = 2000 + (int64_t)dT * C[6] / 8388608;
    OFF = (int64_t)C[2] * 65536 + dT * (int64_t)C[4] / 128;
    SENS = (int64_t)C[1] * 32768 + dT * (int64_t)C[3] / 256;

    // 二阶温度补偿
    if (_compensation) {
        if (TEMP < 2000) {  //  TEMP < -15℃
            int32_t aux = (2000 - TEMP) * (2000 - TEMP);
            int32_t T2 = (int64_t)dT * dT / 2147483647;
            int64_t OFF2 = (5 * aux) / 2;
            int64_t SENS2 = (5 * aux) / 4;

            if (TEMP < -1500) {  //  TEMP < -15℃
                aux = (TEMP + 1500) * (TEMP + 1500);
                OFF2 += 7 * aux;
                SENS2 += (11 * aux) / 2;
            }
            TEMP -= T2;
            OFF -= OFF2;
            SENS -= SENS2;
        }
    }
    P = (int32_t)((D1 * SENS / 2097152 - OFF) / 32768);

    // 计算温度与气压
    _temperature = (double)TEMP * 0.01 + _temperatureOffset;
    _pressure = (double)P * 0.01 + _pressureOffset;

    // 采样时间
    _preRead = _lastRead;
    _lastRead = millis();

    // 清除标志位
    flag_getHeight = false;
    flag_getRelativeHeight = false;
    return MS5611_READ_OK;
}

double MS5611_Base::getHeight(h_mode mode) {
    if (!flag_getHeight) {
        switch (mode) {
        case ONLY_PRESSURE:
            _height = 44330.0 * (1.0 - pow(_pressure / 1013.25, 1 / 5.255));  // barometric
            break;
        case MIXED:
            _height = (pow((1013.25 / _pressure), 1.0 / 5.257) - 1) * (_temperature + 273.15) / 0.65;  // hypsometric
            break;
        default:
            Serial.println("getHeight() - unknown mode(the mode must be one of ONLY_PRESSURE, MIXED)");
            return -1;
        }
        flag_getHeight = true;
    }
    return _height;
}

void MS5611_Base::init(uint32_t delay_time, uint32_t n, h_mode mode) {
    for (int i = 0; i < n; i++) {
        read();
        _T0 += _temperature;
        _P0 += _pressure;
        _H0 += getHeight(mode);
        // Serial.print("Temperature: ");
        // Serial.print(_temperature, 2);
        // Serial.print(" C, Pressure: ");
        // Serial.print(_pressure, 2);
        // Serial.println(" mBar");
        delay(delay_time);
    }
    _T0 /= n;
    _P0 /= n;
    _H0 /= n;
    Serial.println("MS5611 Initialization completed!");
    Serial.printf("T0: %.2f C, P0: %.2f mBar, H0: %.2f m\n", _T0, _P0, _H0);
}

double MS5611_Base::getInit(const std::string& data) {
    if (data == "T0")
        return _T0;
    else if (data == "P0")
        return _P0;
    else if (data == "H0")
        return _H0;
    else {
        Serial.println("getInit() - unknown data(the data must be one of T0, P0, H0)");
        return -1;
    }
}

double MS5611_Base::getRelativeHeight(h_mode mode) {
    if (!flag_getRelativeHeight) {
        double relative_height = getHeight(mode) - _H0;
        _height_filter = H_filter.Butterworth50HzLPF(relative_height);
        flag_getRelativeHeight = true;
    }
    return _height_filter;
}

void MS5611_Base::list() const {
    for (uint8_t reg = 0; reg < 8; ++reg) {
        Serial.printf("C%d", reg);
        Serial.print(": ");
        Serial.println(C[reg]);
    }
    Serial.printf("D1: %lu\n", D1);
    Serial.printf("D2: %lu\n", D2);
    Serial.printf("dT: %ld\n", dT);
    Serial.printf("TEMP: %ld\n", TEMP);
    Serial.printf("OFF: %lld\n", OFF);
    Serial.printf("SENS: %lld\n", SENS);
    Serial.printf("P: %ld\n", P);
    Serial.printf("Temperature: %.2f\n", _temperature);
    Serial.printf("Pressure: %.2f\n", _pressure);
}

void MS5611_Base::print(uint32_t delay_time) {
    auto start_ms5611 = millis();
    read();
    auto stop_ms5611 = millis();
    Serial.print("温度: ");
    Serial.print(_temperature, 2);
    Serial.print(" ℃, 气压: ");
    Serial.print(_pressure, 2);
    Serial.print(" mBar, 相对高度: ");
    Serial.print(getRelativeHeight(), 2);
    Serial.print(" m, 单次采样用时: ");
    Serial.print(stop_ms5611 - start_ms5611);
    Serial.print(" ms, 两次采样间隔: ");
    Serial.print(getTimeBetweenRead());
    Serial.println(" ms");
    delay(delay_time);
}

// protected
void MS5611_Base::convert(const uint8_t addr, osr_t overSamplingRate) {
    // ADC转换时间与过采样率的关系
    uint16_t delay[5] = {600, 1200, 2300, 4600, 9100};

    uint8_t index = 0;
    switch (overSamplingRate) {
    case OSR_LOW:
        index = 0;
        break;
    case OSR_ULTRA_LOW:
        index = 1;
        break;
    case OSR_STANDARD:
        index = 2;
        break;
    case OSR_HIGH:
        index = 3;
        break;
    case OSR_ULTRA_HIGH:
        index = 4;
        break;
    default:
        break;
    }
    uint8_t offset = index * 2;
    command(addr + offset);

    // 等待ADC转换完成
    uint16_t waitTime = delay[index];
    uint32_t start = micros();
    //  while loop prevents blocking RTOS
    while (micros() - start < waitTime) {
        yield();
        delayMicroseconds(10);
    }
}

bool MS5611_Base::checkCRC() {
    uint8_t zero = 1;          // 标志位，用于检查EEPROM是否全为零
    uint32_t res = 0;          // 存储计算出的CRC值
    uint8_t crc = C[7] & 0xF;  // 从传入的校准值数组的最后一个元素中提取原始CRC值。

    C[7] &= 0xFF00;  // 清除C[7]中的CRC值，以便进行计算

    // 检查EEPROM是否全为零
    for (uint8_t i = 0; i < 8; ++i) {
        if (C[i] != 0) {
            zero = 0;
            break;
        }
    }
    if (zero) return false;

    // 每个C[i]元素被分为两个字节处理，所以总共处理16个字节
    for (uint8_t i = 0; i < 16; ++i) {
        if (i & 1)
            res ^= ((C[i >> 1]) & 0x00FF);
        else
            res ^= (C[i >> 1] >> 8);
        // 通过左移和条件异或操作来处理每个位的CRC计算
        for (uint8_t j = 8; j > 0; --j) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    // 恢复原始CRC值
    C[7] |= crc;
    if (crc == ((res >> 12) & 0xF))
        return true;
    return false;
}
