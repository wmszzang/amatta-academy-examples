#include <fstream>
#include "quat.hpp"
int main() {
    std::ofstream f("angles.svg");
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
         "<rect width='1200' height='800' fill='#F4F6FB'/>"
         "<path d='M100 70V700H1100' fill='none' stroke='#1E293B' stroke-width='3'/>"
         "<text x='110' y='40' font-size='26'>Rotation angle (0 to 170 degrees)</text>"
         "<text x='850' y='750' font-size='26'>Progress t (0 to 1)</text>";
    Quat I={1,0,0,0},E=axis_deg(0,0,1,170);
    for(int mode=0;mode<2;mode++) {
        f << "<polyline fill='none' stroke='" << (mode?"#D97706":"#4F46E5") << "' stroke-width='5' points='";
        for(int i=0;i<=100;i++) {double t=i/100.;double a=geodesic_deg(I,mode?nlerp(I,E,t):slerp(I,E,t));f<<100+1000*t<<","<<700-a/170*600<<" ";}
        f << "'/>";
    }
    f << "<text x='150' y='130' fill='#4F46E5' font-size='28'>SLERP</text>"
         "<text x='150' y='180' fill='#D97706' font-size='28'>Normalized LERP</text></svg>";
    return !f;
}
