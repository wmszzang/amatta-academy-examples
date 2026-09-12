#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

void fk(const double *theta, const double *length, int n, double *px, double *py) {
    double acc = 0.0, x = 0.0, y = 0.0;
    px[0] = py[0] = 0.0;
    for (int i = 0; i < n; ++i) {
        acc += theta[i] * PI / 180.0;
        x += length[i] * cos(acc);
        y += length[i] * sin(acc);
        px[i + 1] = x; py[i + 1] = y;
    }
}

int main(void) {
    const double theta[] = {30, 45}, length[] = {1.5, 1.0};
    double px[3], py[3];
    fk(theta, length, 2, px, py);
    for (int i = 0; i < 3; ++i) printf("(%.4f,%.4f)%s", px[i], py[i], i == 2 ? "\n" : " ");
    return 0;
}
