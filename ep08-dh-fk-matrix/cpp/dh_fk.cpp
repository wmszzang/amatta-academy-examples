#include <cmath>
#include <iomanip>
#include <iostream>

struct T4 { double m[4][4]; };

T4 dh(double a, double alpha, double d, double theta) {
    double ca=std::cos(alpha), sa=std::sin(alpha), ct=std::cos(theta), st=std::sin(theta);
    return {{{ct,-st*ca,st*sa,a*ct},{st,ct*ca,-ct*sa,a*st},{0,sa,ca,d},{0,0,0,1}}};
}

T4 mul(const T4& a, const T4& b) {
    T4 out{};
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) for (int k=0;k<4;++k) out.m[i][j]+=a.m[i][k]*b.m[k][j];
    return out;
}

int main() {
    auto t=mul(dh(2,0,0,0.5236),dh(1.5,0,0,0.7854));
    std::cout<<std::fixed<<std::setprecision(4)<<"POSE "<<t.m[0][3]<<","<<t.m[1][3]<<","<<t.m[2][3]<<"\n";
    for (int angle=0;angle<=90;angle+=15) {
        double r=angle*0.01745329252;
        std::cout<<std::setw(2)<<std::setfill('0')<<angle<<" "<<2*std::cos(0.5235987756)+1.5*std::cos(0.5235987756+r)<<","<<2*std::sin(0.5235987756)+1.5*std::sin(0.5235987756+r)<<"\n";
    }
}
