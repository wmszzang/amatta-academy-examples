// 그림 좌표는 계산 코어의 부모 장부와 최종 경로에서 가져온다.
#include <cstdlib>
#include <fstream>
#include <iomanip>
#define PATH_WRAPUP_LIBRARY
#include "../c/path_wrapup.c"
int main(){
    verify(); Tree tree=sampling(samples,7,1,0);
    std::ofstream out("path_wrapup.svg");
    out<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'><rect width='1200' height='800' fill='#f4f6fb'/><g font-family='Arial' fill='#1e293b'><text x='75' y='60' font-size='32'>RRT* - computed path and descendant costs</text>";
    out<<"<circle cx='540' cy='510' r='140' fill='#ffe4e6' stroke='#e11d48' stroke-width='3'/>";
    for(int i=1;i<tree.count;i++){Node n=tree.nodes[i],p=tree.nodes[n.parent];out<<"<line x1='"<<260+p.point.x*140<<"' y1='"<<510-p.point.y*140<<"' x2='"<<260+n.point.x*140<<"' y2='"<<510-n.point.y*140<<"' stroke='#a5b4fc' stroke-width='4'/>";}
    for(int j=1;j<tree.np;j++){Point a=tree.nodes[tree.path[j-1]].point,b=tree.nodes[tree.path[j]].point;out<<"<line x1='"<<260+a.x*140<<"' y1='"<<510-a.y*140<<"' x2='"<<260+b.x*140<<"' y2='"<<510-b.y*140<<"' stroke='#4f46e5' stroke-width='8'/>";}
    const char *names[]={"S","A","B","D","G","E","N"};
    for(int i=0;i<tree.count;i++){Node n=tree.nodes[i];out<<"<circle cx='"<<260+n.point.x*140<<"' cy='"<<510-n.point.y*140<<"' r='7' fill='#4f46e5'/><text x='"<<273+n.point.x*140<<"' y='"<<500-n.point.y*140<<"' font-size='20'>"<<names[i]<<" "<<std::fixed<<std::setprecision(4)<<n.cost<<"</text>";}
    out<<"<text x='75' y='735' font-size='28'>S - N - G: "<<std::setprecision(9)<<tree.nodes[4].cost<<" m</text></g></svg>";
}
