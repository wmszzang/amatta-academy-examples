#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

int main(int argc,char **argv){
    std::string dir=argc>1?argv[1]:".",line;
    std::ifstream points(dir+"/workspace.csv"),hull(dir+"/hull.csv");
    if(!points||!hull){std::cerr<<"Run workspace first.\n";return 1;}
    std::ofstream out(dir+"/workspace.svg");
    out<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'><rect width='1200' height='800' fill='#f4f6fb'/><g transform='translate(600,400) scale(95,-95)'>";
    std::getline(points,line);
    while(std::getline(points,line)){
        std::istringstream row(line);double x,y,w,k;char sep;
        if(!(row>>x>>sep>>y>>sep>>w>>sep>>k))return 2;
        out<<"<circle cx='"<<x<<"' cy='"<<y<<"' r='.018' fill='#4f46e5'/>";
    }
    out<<"<polygon fill='none' stroke='#e11d48' stroke-width='.025' points='";
    std::getline(hull,line);
    while(std::getline(hull,line)){std::istringstream row(line);double x,y;char sep;row>>x>>sep>>y;out<<x<<","<<y<<" ";}
    out<<"'/></g></svg>";
    std::cout<<dir<<"/workspace.svg\n";return out?0:1;
}
