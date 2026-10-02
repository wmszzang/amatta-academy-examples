#include <stdio.h>
#include <string.h>
int project(const double X[3],const double R[3][3],const double t[3],double fx,double fy,double cx,double cy,double uv[2]){
    double p[3];for(int i=0;i<3;i++){p[i]=t[i];for(int j=0;j<3;j++)p[i]+=R[i][j]*X[j];}
    if(p[2]<=0)return 0;
    uv[0]=fx*p[0]/p[2]+cx;uv[1]=fy*p[1]/p[2]+cy;return 1;
}
int main(int argc,char**argv){
    double X[3]={1,2,5},t[3]={.5,-.5,1},uv[2];const double R[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    if(argc>1 && strcmp(argv[1],"--invalid")==0)X[2]=-1;
    if(!project(X,R,t,500,500,320,240,uv)){fprintf(stderr,"ERROR: nonpositive camera depth\n");return 2;}
    printf("(%.4f, %.4f)\n",uv[0],uv[1]);return 0;
}
