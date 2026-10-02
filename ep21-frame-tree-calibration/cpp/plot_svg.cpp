#include <fstream>
int main(){
    std::ofstream f("result.svg");
    f<<"<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'><rect width='1200' height='800' fill='#f4f6fb'/>";
    f<<"<text x='80' y='80' font-size='36' fill='#1e293b'>Q to P: 90 degrees, t=(2,3)</text>";
    const double Q[4][2]={{0,0},{1,0},{1,1},{0,1}};
    for(int i=0;i<4;i++){
        double x=-Q[i][1]+2,y=Q[i][0]+3;
        f<<"<circle cx='"<<200+Q[i][0]*100<<"' cy='"<<650-Q[i][1]*100<<"' r='12' fill='#4f46e5'/>";
        f<<"<circle cx='"<<200+x*100<<"' cy='"<<650-y*100<<"' r='12' fill='#059669'/>";
    }
    f<<"</svg>";return !f.good();
}
