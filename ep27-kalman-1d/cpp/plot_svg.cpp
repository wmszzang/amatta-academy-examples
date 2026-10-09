#include <fstream>
#include <sstream>
#include <string>
int main(){
    std::ifstream in("stationary.csv"); if(!in) return 1;
    std::ofstream out("filters.svg");
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1200' height='800' viewBox='0 0 1200 800'><rect width='1200' height='800' fill='#F4F6FB'/><path d='M100 100V680H1120' fill='none' stroke='#1E293B' stroke-width='3'/><text x='110' y='70' font-size='28'>Stationary Kalman: measurements and estimate (initial = 0)</text><path d='M100 265H1100' stroke='#059669' stroke-dasharray='10 8'/><polyline fill='none' stroke='#4F46E5' stroke-width='4' points='100,680 ";
    std::string line;std::getline(in,line); std::ostringstream dots;
    while(std::getline(in,line)){
        std::stringstream s(line);std::string cell;double v[9];int j=0;
        while(std::getline(s,cell,','))v[j++]=std::stod(cell);
        out << 100+v[0]*65 << "," << 680-v[7]*83 << " ";
        dots << "<circle cx='" << 100+v[0]*65 << "' cy='" << 680-v[2]*83 << "' r='6' fill='#D97706'/>";
    }
    out << "'/>" << dots.str();
    for(int k=0;k<=15;k+=3) out << "<text x='" << 95+65*k << "' y='710' font-size='20'>" << k << "</text>";
    for(int k=0;k<=6;k++) out << "<text x='65' y='" << 687-83*k << "' font-size='20'>" << k << "</text>";
    out << "<text x='150' y='120' font-size='22' fill='#D97706'>Measurement</text><text x='410' y='120' font-size='22' fill='#4F46E5'>Estimate</text><text x='620' y='120' font-size='22' fill='#059669'>Reference truth = 5 (not input)</text><text x='460' y='755' font-size='24'>Measurement index</text></svg>";
}
