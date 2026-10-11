// EP.29: 계산과 파일 저장을 분리하여 두 함수가 각각 vector를 반환한다.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
struct Sample { double t,x,v,a; };
struct Point { double x,y; };

std::vector<Sample> trapezoidal_profile(double distance=10.,double max_speed=2.,double acceleration=1.,double dt=.25) {
    if(std::min(std::min(distance,max_speed),std::min(acceleration,dt))<=0) throw std::invalid_argument("positive inputs required");
    double peak=std::min(max_speed,std::sqrt(distance*acceleration)), ta=peak/acceleration;
    double da=.5*acceleration*ta*ta, cruise=std::max(0.,(distance-2*da)/peak);
    double t2=ta+cruise,total=2*ta+cruise;
    size_t intervals=static_cast<size_t>(std::ceil(total/dt));
    std::vector<Sample> rows;
    for(size_t i=0;i<=intervals;++i) {
        double t=i==intervals?total:i*dt;
        if(i<intervals && t>=total-1e-12) continue;
        Sample row={t,0,0,0};
        if(t<ta) { row.x=.5*acceleration*t*t; row.v=acceleration*t; row.a=acceleration; }
        else if(t<t2) { row.x=da+peak*(t-ta); row.v=peak; }
        else {double remaining=total-t; row.x=distance-.5*acceleration*remaining*remaining; row.v=acceleration*remaining; row.a=-acceleration;}
        rows.push_back(row);
    }
    return rows;
}
std::vector<Point> transform_points(const std::vector<Point>& points,double degrees=30.,double tx=1.,double ty=.5) {
    double angle=degrees*std::acos(-1.)/180.,c=std::cos(angle),s=std::sin(angle);
    std::vector<Point> result;
    for(const auto& p:points) result.push_back({c*p.x-s*p.y+tx,s*p.x+c*p.y+ty});
    return result;
}
int main(int argc,char** argv) {
    double distance=argc>1?std::stod(argv[1]):10.,dt=argc>2?std::stod(argv[2]):.25;
    try {
        auto rows=trapezoidal_profile(distance,2.,1.,dt);
        std::vector<Point> original={{0,0},{1,0},{1,1},{0,1}};
        auto points=transform_points(original);
        std::ofstream trajectory("trajectory.csv"),vertices("points.csv");
        if(!trajectory || !vertices) return 1;
        trajectory<<std::fixed<<std::setprecision(10)<<"t,x,v,a\n";
        for(const auto& r:rows) trajectory<<r.t<<','<<r.x<<','<<r.v<<','<<r.a<<'\n';
        vertices<<std::fixed<<std::setprecision(10)<<"id,x,y,xp,yp\n";
        for(size_t i=0;i<points.size();++i) vertices<<char('A'+i)<<','<<original[i].x<<','<<original[i].y<<','<<points[i].x<<','<<points[i].y<<'\n';
        const auto& last=rows.back();
        std::cout<<std::fixed<<std::setprecision(10)<<"rows="<<rows.size()<<" T="<<last.t<<" x="<<last.x<<" v="<<last.v<<" a="<<last.a<<'\n';
        for(size_t i=0;i<points.size();++i) std::cout<<char('A'+i)<<' '<<points[i].x<<' '<<points[i].y<<'\n';
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
