#include <cmath>
#include <fstream>
#include <iomanip>

int main() {
    std::ofstream f("jacobian.svg");
    if (!f) return 1;
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='675' viewBox='0 0 1200 675'>"
      << "<rect width='1200' height='675' fill='#f4f6fb'/>"
      << "<g font-family='sans-serif' fill='#1e293b'><text x='70' y='55' font-size='30'>EP.11: radial command 0.10 m/s</text>"
      << "<text x='70' y='110' font-size='24'>det J</text><text x='670' y='110' font-size='24'>joint speed norm (log10)</text>"
      << "<text x='70' y='625' font-size='20'>theta2: 0.001 to 0.7854 rad</text>"
      << "<text x='670' y='625' font-size='20'>theta2: 0.001 to 0.7854 rad</text></g>";
    for(int panel=0;panel<2;panel++) {
        f << "<path d='M " << (70+600*panel) << " 140 V 570 H " << (550+600*panel) << "' fill='none' stroke='#64748b'/>";
        f << "<polyline fill='none' stroke='" << (panel ? "#0d9488" : "#4f46e5") << "' stroke-width='4' points='";
        for(int i=0;i<=180;i++) {
            double t=.001+(.7854-.001)*i/180, t1=std::acos(-1.0)/6;
            double px=2*std::cos(t1)+1.5*std::cos(t1+t),py=2*std::sin(t1)+1.5*std::sin(t1+t);
            double b=-1.5*std::sin(t1+t),d=1.5*std::cos(t1+t),det=3*std::sin(t);
            double vx=.1*px/std::hypot(px,py),vy=.1*py/std::hypot(px,py);
            double q1=(d*vx-b*vy)/det,q2=(-px*vx-py*vy)/det;
            double val=panel?(std::log10(std::hypot(q1,q2))+1)/3.2:det/2.2;
            f << (70+600*panel+480.0*i/180) << ',' << (570-410*val) << ' ';
        }
        f << "'/>";
    }
    f << "</svg>";
    return 0;
}
