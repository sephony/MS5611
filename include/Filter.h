#ifndef FILTER_H_
#define FILTER_H_

#define GAINBtw50hz (3.450423889e+02f)  // 50Hz Butterworth LPF

class Filter {
public:
    Filter() = default;
    float Butterworth50HzLPF(float data);

private:
    float xv[4] = {0, 0, 0, 0};
    float yv[4] = {0, 0, 0, 0};
    float output = 0;
};

#endif /* FILTER_H_ */
