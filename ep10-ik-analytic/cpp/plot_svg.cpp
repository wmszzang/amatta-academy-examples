#include <cmath>
#include <fstream>
int main(){
std::ofstream f("ik2link.svg");
f<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800'><rect width='1200' height='800' fill='#f4f6fb'/><circle cx='350' cy='530' r='280' fill='#eef2ff'/><circle cx='350' cy='530' r='40' fill='white'/>";
for(int e=1;e>=-1;e-=2){double s=e*std::sqrt(1-.375*.375),t=std::atan2(1.5,2.5)-std::atan2(1.5*s,2+1.5*.375);
double ex=350+160*std::cos(t),ey=530-160*std::sin(t);
f<<"<polyline points='350,530 "<<ex<<","<<ey<<" 550,410' fill='none' stroke='"<<(e==1?"#4f46e5":"#059669")<<"' stroke-width='12'/><circle cx='"<<ex<<"' cy='"<<ey<<"' r='10' fill='#1e293b'/>";}
f<<"<circle cx='550' cy='410' r='10' fill='#e11d48'/><text x='300' y='100' font-size='32' fill='#1e293b'>C++: two solutions, one target</text></svg>";}
