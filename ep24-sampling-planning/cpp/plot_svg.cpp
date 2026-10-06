// 수치 코어를 공유해 그림과 콘솔의 입력·계산 규칙을 일치시킨다.
#define main numerical_example_main
#include "../c/sampling_planning.c"
#undef main
#include <fstream>
#include <iomanip>

int main() {
    const Point samples[]={{4,0},{0,2},{2,2},{4,2},{4,0}};
    Result result=rrt(samples,5);
    std::ofstream out("sampling_planning.svg");
    if(!out) return 1;
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'>"
        << "<rect width='1200' height='800' fill='#F4F6FB'/>"
        << "<g font-family='Arial' fill='#1E293B' font-size='25'>"
        << "<text x='50' y='55'>EP.24 - verified C++ execution</text>"
        << "<text x='50' y='115'>RRT: fixed samples, recovered path</text>"
        << "<circle cx='300' cy='410' r='85' fill='#FDF6E3' stroke='#E11D48' stroke-width='3'/>";
    for(int i=1;i<result.length;i++) {
        Point a=result.path[i-1],b=result.path[i];
        out << "<line x1='" << 130+a.x*85 << "' y1='" << 410-a.y*85
            << "' x2='" << 130+b.x*85 << "' y2='" << 410-b.y*85
            << "' stroke='#4F46E5' stroke-width='7'/>";
    }
    for(int i=0;i<result.length;i++) {
        Point p=result.path[i];
        out << "<circle cx='" << 130+p.x*85 << "' cy='" << 410-p.y*85 << "' r='7' fill='#4F46E5'/>"
            << "<text x='" << 110+p.x*85 << "' y='" << 450-p.y*85 << "'>(" << p.x << "," << p.y << ")</text>";
    }
    double length=0;
    for(int i=1;i<result.length;i++)length+=distance(result.path[i-1],result.path[i]);
    out << "<text x='50' y='540'>Path length = " << length << " (not shortest-path proof)</text>"
        << "<text x='650' y='115'>#401 candidate scores</text>";
    const double v[]={.5,.5,.3},w[]={0,.3,0},d[]={2,1.5,2.5};
    double best=-1; int choice=-1;
    for(int i=0;i<3;i++){
        double angle=fabs(w[i]*.1),score=.4/(1+angle)+.2*v[i]+.4*fmin(d[i]/3,1);
        if(score>best){best=score;choice=i;}
        out << "<rect x='650' y='" << 170+i*90 << "' width='" << score*480 << "' height='30' fill='" << (i==2?"#059669":"#4F46E5") << "'/>"
            << "<text x='650' y='" << 155+i*90 << "'>(" << v[i] << "," << w[i] << ") = " << std::fixed << std::setprecision(6) << score << "</text>";
    }
    out << "<text x='650' y='470'>Selected candidate " << choice+1 << "</text>"
        << "<text x='650' y='515'>Score only; safety not evaluated</text>";
    const int grid[]={0,0,0,0,1,0}; Cell cells[256]; int n=coverage(grid,2,3,cells);
    out << "<text x='50' y='640'>#505 visit list: ";
    for(int i=0;i<n;i++)out << (i?" / ":"") << "(" << cells[i].r << "," << cells[i].c << ")";
    out << "</text><text x='50' y='700'>A visit list is not a continuous collision-free travel path.</text></g></svg>";
    return out.good()?0:1;
}
