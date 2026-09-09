#include <fstream>
int main(){std::ofstream f("frames.svg");f<<"<svg xmlns='http://www.w3.org/2000/svg' width='600' height='400'><line x1='300' y1='200' x2='500' y2='200' stroke='red' stroke-width='5'/><line x1='300' y1='200' x2='300' y2='40' stroke='green' stroke-width='5'/></svg>";}
