#include <stdio.h>

static void kalman(const char *name, const double *z, const double *u, int n, double q, double r) {
    double x=0, p=1; char file[80], line[512];
    snprintf(file,sizeof(file),"%s.csv",name);
    FILE *f=fopen(file,"w");
    if (!f) return;
    fprintf(f,"k,u,z,x_pred,P_pred,K,residual,x,P\n");
    for (int i=0;i<n;i++) {
        double xp=x+u[i], pp=p+q, gain=pp/(pp+r), residual=z[i]-xp;
        x=xp+gain*residual; p=(1-gain)*pp;
        snprintf(line,sizeof(line),"%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",i+1,u[i],z[i],xp,pp,gain,residual,x,p);
        fprintf(f,"%s\n",line);
    }
    fclose(f); printf("%s: %s\n",name,line);
}
int main(void) {
    const double z[]={4.2,6.1,4.8,5.5,4.3,5.9,5.1,4.7,5.3,4.9,5.2,4.6,5.4,5.0,4.8};
    const double u[15]={0}, zm[]={1.2,2.1}, um[]={1,1};
    kalman("stationary",z,u,15,0.01,1);
    kalman("moving",zm,um,2,0.1,0.5);
    FILE *f=fopen("complementary.csv","w");
    if (!f) return 1;
    fprintf(f,"k,t,gyro,accel,predicted,theta\n");
    double theta=0; char line[512];
    for (int k=1;k<=20;k++) {
        double t=k*0.1, angle=10*t, predicted=theta+12*0.1;
        theta=0.95*predicted+0.05*angle;
        snprintf(line,sizeof(line),"%d,%.6f,%.6f,%.6f,%.6f,%.6f",k,t,12*t,angle,predicted,theta);
        fprintf(f,"%s\n",line);
    }
    fclose(f); printf("complementary: %s\n",line); return 0;
}
