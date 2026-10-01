// 별도 그래픽 라이브러리 없이 동일 축척 SVG를 만든다.
#include <cmath>
#include <fstream>
#include <iomanip>
int main() {
    std::ofstream f("trajectory.svg");
    if(!f) return 1;
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
      << "<rect width='1200' height='800' fill='#F4F6FB'/>"
      << "<path d='M350 500 H900 M350 500 V100' fill='none' stroke='#1E293B' stroke-width='2'/>"
      << "<text x='880' y='540' font-size='24'>x (m)</text><text x='260' y='130' font-size='24'>y (m)</text>"
      << "<polyline fill='none' stroke='#4F46E5' stroke-width='4' points='350,500 ";
    double x=0,y=0,th=0;
    for(int i=0;i<40;++i) {
        x+=.11*std::cos(th+.02);y+=.11*std::sin(th+.02);th+=.04;
        f<<std::fixed<<std::setprecision(6)<<350+120*x<<","<<500-120*y<<" ";
    }
    f << "'/><text x='350' y='65' font-size='28'>#224: 41 points, midpoint angle</text></svg>";
    return f.good()?0:1;
}
