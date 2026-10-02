#include <stdio.h>
#include <string.h>
int triangulate(double uL,double vL,double uR,double fx,double B,double cx,double cy,double p[3]){
    double d=uL-uR;if(d<=0 || fx<=0 || B<=0)return 0;
    p[2]=fx*B/d;p[0]=(uL-cx)*p[2]/fx;p[1]=(vL-cy)*p[2]/fx;return 1;
}
int main(int argc,char**argv){
    double p[3],uR=345;
    if(argc>1 && strcmp(argv[1],"--invalid")==0)uR=370;
    if(!triangulate(370,240,uR,500,.1,320,240,p)){fprintf(stderr,"ERROR: nonpositive disparity\n");return 2;}
    printf("(%.4f, %.4f, %.4f)\n",p[0],p[1],p[2]);return 0;
}
