#ifndef FILTER_H_
#define FILTER_H_

#define GAINBtw50hz (3.450423889e+02f)  // 50Hz Butterworth LPF

class Filter {
public:
    Filter() = default;
    /*
     * @brief 50Hz的Butterworth低通滤波器
     * @param data 原始数据
     * @return
     * @details Butterworth滤波器是一种在通带内具有平坦频率响应的滤波器，通常用于消除高频噪声。
     *
     *       滤波器的工作原理如下：
     *
     *       1. 首先，滤波器的输入值`data`被除以一个增益值`GAINBtw50hz`，然后存储在
     *       `this->xv[3]`中。同时，`this->xv`数组中的其他元素被向前移动一位，以便为新的输入值腾出空间。
     *
     *       2. 然后，滤波器的输出值`this->yv[3]`被计算为`this->xv`和`this->yv`数组中
     *       的元素的加权和。这个加权和的权重是固定的，由Butterworth滤波器的设计决定。
     *
     *       3. 最后，计算出的输出值`this->yv[3]`被存储在`this->output`中，并作为函数的返回值。

     *       在这个过程中，高频噪声被滤除，因为它们在加权和中的权重较小，而低频信号被保留，
     *       因为它们在加权和中的权重较大。
     */
    double Butterworth50HzLPF(double data);

private:
    double xv[4] = {0, 0, 0, 0};
    double yv[4] = {0, 0, 0, 0};
    double output = 0;
};

#endif /* FILTER_H_ */
