#include <array>
#include <cstdio>
using M4=std::array<std::array<double,4>,4>;
M4 mul(const M4&A,const M4&B){M4 C{};for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)C[i][j]+=A[i][k]*B[k][j];return C;}
int main(){M4 A={{{0,-1,0,1},{1,0,0,0},{0,0,1,0},{0,0,0,1}}},B={{{1,0,0,0},{0,1,0,2},{0,0,1,0},{0,0,0,1}}};auto C=mul(A,B);std::printf("t_AC=(%.4f, %.4f, %.4f) p_A=(-1.0000, 1.0000, 0.0000)\n",C[0][3],C[1][3],C[2][3]);}
