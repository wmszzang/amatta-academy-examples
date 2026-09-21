#include <fstream>
#include <cmath>
#include <string>

int main(int argc, char **argv) {
    std::ofstream f(argc>1?argv[1]:"projection.svg");
    if (!f) return 1;
    const double k[3]={1.0/3,2.0/3,2.0/3};
    const double t=std::acos(-1.0)/3;
    const double x=std::cos(t)+(1-std::cos(t))*k[0]*k[0];
    const double y=std::sin(t)*k[2]+(1-std::cos(t))*k[1]*k[0];
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
         "<rect width='1200' height='800' fill='#F4F6FB'/>"
         "<defs><marker id='tip' markerWidth='10' markerHeight='10' refX='8' refY='3' orient='auto'><path d='M0,0 L0,6 L9,3 z' fill='context-stroke'/></marker></defs>"
         "<g stroke='#94A3B8' stroke-width='2'><line x1='250' y1='500' x2='970' y2='500'/><line x1='450' y1='680' x2='450' y2='100'/></g>";
    const double vectors[3][2]={{1,0},{x,y},{k[0],k[1]}};
    const char *colors[]={"#D97706","#4F46E5","#059669"};
    const char *labels[]={"v: before","Rv: after 60 deg","k: axis (xy projection)"};
    for (int i=0;i<3;i++) {
        double px=450+400*vectors[i][0],py=500-400*vectors[i][1];
        f << "<line x1='450' y1='500' x2='" << px << "' y2='" << py
          << "' stroke='" << colors[i] << "' stroke-width='6' marker-end='url(#tip)'/>";
        f << "<text x='" << (i==2?120:780) << "' y='" << (i==0?555:(i==1?180:100))
          << "' font-family='sans-serif' font-size='25' fill='" << colors[i] << "'>" << labels[i] << "</text>";
    }
    f << "<text x='50' y='755' font-family='sans-serif' font-size='25' fill='#1E293B'>XY projection only: z is omitted. k=(1,2,2)/3; angle=60 deg.</text></svg>";
}
