// 쿼터니언 구면의 w-z 단면: 호와 현을 실제 좌표로 그린다.
#include <fstream>
#include <cmath>
#include <iomanip>
int main(){
    std::ofstream f("slerp.svg");
    const double pi=3.14159265358979323846,r=230,cx=330,cy=350;
    f<<"<svg xmlns='http://www.w3.org/2000/svg' width='1000' height='650' viewBox='0 0 1000 650'>"
      <<"<rect width='1000' height='650' fill='#f4f6fb'/>"
      <<"<g font-family='sans-serif' fill='#1e293b'><text x='50' y='55' font-size='30'>SLERP: quaternion w-z section</text>"
      <<"<text x='50' y='605' font-size='22'>Physical rotation 90 deg = quaternion arc 45 deg</text></g>"
      <<"<circle cx='330' cy='350' r='230' fill='none' stroke='#cbd5e1' stroke-width='3'/>";
    f<<"<polyline fill='none' stroke='#4f46e5' stroke-width='7' points='";
    for(int i=0;i<=100;i++){double a=pi/4*i/100;f<<cx+r*cos(a)<<","<<cy-r*sin(a)<<" ";}f<<"'/>";
    f<<"<line x1='560' y1='350' x2='"<<cx+r*cos(pi/4)<<"' y2='"<<cy-r*sin(pi/4)<<"' stroke='#d97706' stroke-width='4'/>";
    for(int i=0;i<=4;i++){double a=pi/4*i/4;f<<"<circle cx='"<<cx+r*cos(a)<<"' cy='"<<cy-r*sin(a)<<"' r='7' fill='#4f46e5'/>";}
    f<<"<g font-family='sans-serif' font-size='24'><text x='640' y='240' fill='#4f46e5'>SLERP: arc</text>"
      <<"<text x='640' y='300' fill='#d97706'>LERP: chord</text></g></svg>";
    return f?0:1;
}
