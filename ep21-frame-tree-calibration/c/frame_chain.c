#include <stdio.h>
void apply_T(const double R[3][3], const double t[3], const double p[3], double out[3]) {
    for(int i=0;i<3;i++){out[i]=t[i];for(int j=0;j<3;j++)out[i]+=R[i][j]*p[j];}
}
void inv_T(const double R[3][3], const double t[3], double Ri[3][3],double ti[3]) {
    for(int i=0;i<3;i++){ti[i]=0;for(int j=0;j<3;j++){Ri[i][j]=R[j][i];ti[i]-=R[j][i]*t[j];}}
}
void compose_T(const double Ra[3][3], const double ta[3], const double Rb[3][3], const double tb[3], double R[3][3], double t[3]) {
    apply_T(Ra,ta,tb,t);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){
        R[i][j]=0;for(int k=0;k<3;k++)R[i][j]+=Ra[i][k]*Rb[k][j];
    }
}
int main(void){
    const double Ra[3][3]={{1,0,0},{0,1,0},{0,0,1}},ta[3]={.1,0,.2};
    const double Rb[3][3]={{0,0,1},{-1,0,0},{0,-1,0}},tb[3]={.2,0,.2},p[3]={.2,0,2};
    double R[3][3],t[3];compose_T(Ra,ta,Rb,tb,R,t);
    double result[3],back[3],Ri[3][3],ti[3];
    apply_T(R,t,p,result);inv_T(R,t,Ri,ti);apply_T(Ri,ti,result,back);
    printf("p_base = (%.4f, %.4f, %.4f)\n",result[0],result[1],result[2]);
    printf("back = (%.4f, %.4f, %.4f)\n",back[0],back[1],back[2]);return 0;
}
