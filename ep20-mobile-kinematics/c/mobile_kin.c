/* EP.20: 출력할 때만 반올림하며 각도 입력은 라디안을 사용한다. */
#include <math.h>
#include <stdio.h>
#include <assert.h>
const double PI = 3.14159265358979323846;

void diff_ik(double v,double w,double r,double L,double *out) {
    out[0]=v-w*L/2; out[1]=v+w*L/2;
    out[2]=out[0]/r; out[3]=out[1]/r;
}
void diff_fk(double left,double right,double L,double *v,double *w) {
    *v=(left+right)/2; *w=(right-left)/L;
}
void mecanum_ik(double vx,double vy,double w,double r,double lx,double ly,double *out) {
    const double k=lx+ly;
    out[0]=(vx-vy-k*w)/r; out[1]=(vx+vy+k*w)/r;
    out[2]=(vx+vy-k*w)/r; out[3]=(vx-vy+k*w)/r;
}
void mecanum_fk(const double *a,double r,double lx,double ly,double *out) {
    out[0]=r*(a[0]+a[1]+a[2]+a[3])/4;
    out[1]=r*(-a[0]+a[1]+a[2]-a[3])/4;
    out[2]=r*(-a[0]+a[1]-a[2]+a[3])/(4*(lx+ly));
}
int ackermann(double L,double T,double R,double *inner,double *outer) {
    if(L<=0 || T<0 || R<=T/2) return 0;
    *inner=atan(L/(R-T/2))*180/PI;
    *outer=atan(L/(R+T/2))*180/PI;
    return 1;
}
void bicycle_step(double *x,double *y,double *th,double v,double d,double L,double dt) {
    /* 위치 두 값에 갱신 전 방향을 사용한다. */
    *x+=v*cos(*th)*dt; *y+=v*sin(*th)*dt;
    *th+=(v/L)*tan(d)*dt;
}
void odometry(double dL,double dR,double L,int N,int midpoint,double (*pts)[2],double *th) {
    double x=0,y=0,dc=(dL+dR)/2,dth=(dR-dL)/L;
    *th=0; pts[0][0]=0; pts[0][1]=0;
    for(int i=1;i<=N;++i) {
        const double angle=*th+(midpoint?dth/2:0);
        x+=dc*cos(angle); y+=dc*sin(angle); *th+=dth;
        pts[i][0]=x; pts[i][1]=y;
    }
}
void tick_to_dist(int ticks,double r,int CPR,double dt,double *cumulative,double *velocity) {
    const double per=2*PI*r/CPR;
    *cumulative+=ticks*per; *velocity=ticks*per/dt;
}
int pure_pursuit(double gy,double Ld,double L,double *kappa,double *delta) {
    if(Ld<=0) return 0;
    *kappa=2*gy/(Ld*Ld); *delta=atan(*kappa*L);
    return 1;
}

int main(void) {
    double a[4],b[3],inner,outer,x=0,y=0,th=0,pts[41][2],old[41][2],slip[41][2];
    double kappa,delta,cumulative=0,velocity=0,sth=0,unused=0;
    const int ticks[5]={0,100,205,300,412};
    diff_ik(.5,.4,.05,.3,a);
    assert(fabs(a[0]-.44)<1e-12 && fabs(a[3]-11.2)<1e-12);
    printf("261,%.4f,%.4f,%.4f,%.4f\n",a[0],a[1],a[2],a[3]);
    diff_fk(a[0],a[1],.3,&b[0],&b[1]);
    printf("261_fk,%.4f,%.4f\n",b[0],b[1]);
    mecanum_ik(1,.5,.2,.05,.3,.25,a);
    assert(fabs(a[0]-7.8)<1e-12 && fabs(a[1]-32.2)<1e-12);
    printf("483,%.4f,%.4f,%.4f,%.4f\n",a[0],a[1],a[2],a[3]);
    mecanum_fk(a,.05,.3,.25,b);
    assert(fabs(b[0]-1)<1e-12 && fabs(b[1]-.5)<1e-12 && fabs(b[2]-.2)<1e-12);
    printf("483_fk,%.4f,%.4f,%.4f\n",b[0],b[1],b[2]);
    assert(ackermann(2.5,1.5,10,&inner,&outer));
    assert(fabs(inner-15.1240)<.00005 && fabs(outer-13.0919)<.00005);
    printf("481,%.4f,%.4f\n",inner,outer);
    for(int i=0;i<10;++i) bicycle_step(&x,&y,&th,1,.1,2,.1);
    assert(fabs(x-.9996)<.00005 && fabs(y-.0226)<.00005);
    printf("463,%.4f,%.4f,%.4f\n",x,y,th);
    odometry(.1,.12,.5,40,1,pts,&th);
    assert(fabs(pts[40][0]-2.7490)<.00005 && fabs(pts[40][1]-2.8305)<.00005);
    printf("224,%.4f,%.4f,%.4f\n",pts[40][0],pts[40][1],th);
    assert(pure_pursuit(1,5,2.5,&kappa,&delta));
    printf("254,%.4f,%.4f\n",kappa,delta);
    assert(pure_pursuit(0,5,2.5,&kappa,&delta) && kappa==0 && delta==0);
    assert(!pure_pursuit(1,0,2.5,&kappa,&delta));
    for(int i=0;i<5;++i) {
        tick_to_dist(ticks[i],.05,2048,.1,&cumulative,&velocity);
        printf("230_%d,%.4f,%.4f\n",i,cumulative,velocity);
    }
    x=2.75*sin(1.6); y=2.75*(1-cos(1.6));
    printf("exact,%.4f,%.4f\n",x,y);
    odometry(.1,.12,.5,40,0,old,&unused);
    printf("start_angle,%.4f,%.4f,%.4f\n",old[40][0],old[40][1],hypot(old[40][0]-x,old[40][1]-y));
    odometry(.1,.12*.97,.5,40,1,slip,&sth);
    printf("slip,%.4f,%.4f,%.4f,%.4f\n",slip[40][0],slip[40][1],hypot(slip[40][0]-pts[40][0],slip[40][1]-pts[40][1]),(sth-th)*180/PI);
    FILE *f=fopen("trajectory-c.csv","w");
    if(!f) return 1;
    fprintf(f,"step,x_m,y_m\n");
    for(int i=0;i<=40;++i) fprintf(f,"%d,%.4f,%.4f\n",i,pts[i][0],pts[i][1]);
    return fclose(f)!=0;
}
