#include <Filter.h>

double Filter::Butterworth50HzLPF(double data) {
    xv[0] = xv[1];
    xv[1] = xv[2];
    xv[2] = xv[3];
    xv[3] = data / GAINBtw50hz;
    yv[0] = yv[1];
    yv[1] = yv[2];
    yv[2] = yv[3];
    yv[3] = (xv[0] + xv[3]) + 3 * (xv[1] + xv[2]) +
            (0.5320753683f * yv[0]) + (-1.9293556691f * yv[1]) +
            (2.3740947437f * yv[2]);

    output = yv[3];
    return output;
}
