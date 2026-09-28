#include <stdio.h>
#include <math.h>
#include "quat.h"
double clean(double x) {return fabs(x)<.00005 ? 0 : x;}
void show(const char* label,Quat q) {
    printf("%s,%.4f,%.4f,%.4f,%.4f\n",label,clean(q.w),clean(q.x),clean(q.y),clean(q.z));
}
int main(void) {
    Quat a={.7071,0,0,.7071},I={1,0,0,0},j={0,1,0,0};
    Quat Z=axis_deg(0,0,1,90),X=axis_deg(1,0,0,90);
    show("290",qnorm(qmul(a,a)));show("490",qmul(a,a));show("identity",qmul(I,j));
    show("z*x",qmul(Z,X));show("x*z",qmul(X,Z));
    for(int i=1;i<=3;i++) {char label[30];sprintf(label,"259:%.2f",i*.25);show(label,slerp(I,Z,i*.25));}
    Quat targets[]={Z,{-1,0,0,0},{0,0,0,1}};
    for(int i=0;i<3;i++) printf("559,%.4f\n",clean(geodesic_deg(I,targets[i])));
    printf("559,%.4f\n",geodesic_deg(Z,X));
    printf("error,%.4f\n",geodesic_deg(axis_deg(0,0,1,88),Z));
    FILE *f=fopen("angles.csv","w");if(!f)return 1;
    fprintf(f,"t,slerp_deg,nlerp_deg\n");Quat E=axis_deg(0,0,1,170);
    for(int i=0;i<=100;i++) {double t=i/100.;fprintf(f,"%.4f,%.4f,%.4f\n",t,geodesic_deg(I,slerp(I,E,t)),geodesic_deg(I,nlerp(I,E,t)));}
    return fclose(f)!=0;
}
