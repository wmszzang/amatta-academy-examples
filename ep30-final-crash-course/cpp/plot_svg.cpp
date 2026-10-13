// 기존 FK 입력을 계산하여 같은 축 비율의 SVG를 직접 쓴다.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    const double pi = std::acos(-1.0);
    const double x1 = 1.4*std::cos(25*pi/180), y1 = 1.4*std::sin(25*pi/180);
    const double x2 = x1+.9*std::cos(75*pi/180), y2 = y1+.9*std::sin(75*pi/180);
    std::ofstream out("crash_check.svg");
    if (!out) return 1;
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
        << "<rect width='1200' height='800' fill='#f4f6fb'/><g font-family='sans-serif' fill='#1e293b'>"
        << "<text x='70' y='75' font-size='38'>EP.30: C++ FK verification</text>"
        << "<path d='M100 660H1000M100 660V130' stroke='#64748b' stroke-width='3'/>"
        << "<polyline fill='none' stroke='#4f46e5' stroke-width='22' points='100,660 "
        << 100+270*x1 << ',' << 660-270*y1 << ' ' << 100+270*x2 << ',' << 660-270*y2 << "'/>";
    const double xs[] = {0,x1,x2}, ys[] = {0,y1,y2};
    for (int i=0;i<3;i++) out << "<circle cx='" << 100+270*xs[i] << "' cy='" << 660-270*ys[i] << "' r='13' fill='#059669'/>";
    out << std::fixed << std::setprecision(4) << "<text x='700' y='250' font-size='30'>x = " << x2
        << "</text><text x='700' y='300' font-size='30'>y = " << y2
        << "</text><text x='700' y='365' font-size='24'>q1=25 deg; q2=50 deg</text>"
        << "<text x='700' y='410' font-size='24'>L1=1.4; L2=0.9</text>"
        << "<text x='100' y='740' font-size='25'>Equal x/y scale; relative joint angles; no dynamics</text></g></svg>";
    if (!out) return 1;
    std::cout << "crash_check.svg: FK " << x2 << ',' << y2 << '\n';
}
