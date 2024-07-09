#ifndef FILTER_H_
#define FILTER_H_

#define GAINBtw50hz (3.450423889e+02f)  // 50Hz Butterworth LPF

class Filter {
public:
    Filter() = default;
    double Butterworth50HzLPF(double data);

private:
    double xv[4] = {0, 0, 0, 0};
    double yv[4] = {0, 0, 0, 0};
    double output = 0;
};

#endif /* FILTER_H_ */
