#include <math.h>
#include <stdio.h>

void mul(const double a[4][4], const double b[4][4], double out[4][4]) {
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
        out[i][j] = 0.0;
        for (int k = 0; k < 4; ++k) out[i][j] += a[i][k] * b[k][j];
    }
}

void dh(double a, double alpha, double d, double theta, double out[4][4]) {
    double ca = cos(alpha), sa = sin(alpha), ct = cos(theta), st = sin(theta);
    double values[4][4] = {{ct,-st*ca,st*sa,a*ct},{st,ct*ca,-ct*sa,a*st},{0,sa,ca,d},{0,0,0,1}};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) out[i][j] = values[i][j];
}

int main(void) {
    double a[4][4], b[4][4], t[4][4];
    dh(2.0, 0.0, 0.0, 0.5236, a); dh(1.5, 0.0, 0.0, 0.7854, b); mul(a, b, t);
    printf("POSE %.4f,%.4f,%.4f\n", t[0][3], t[1][3], t[2][3]);
    for (int angle = 0; angle <= 90; angle += 15) {
        double x = 2*cos(0.5235987756) + 1.5*cos(0.5235987756 + angle*0.01745329252);
        double y = 2*sin(0.5235987756) + 1.5*sin(0.5235987756 + angle*0.01745329252);
        printf("%02d %.4f,%.4f\n", angle, x, y);
    }
    return 0;
}
