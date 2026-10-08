#include <stdio.h>
#include <math.h>
#include <assert.h>

/* 누적값 J와 이득을 곱한 적분항 I를 구분한다. */
static int position(double kp,double ki,double kd,double dt,int count,double target) {
    if (!isfinite(dt) || dt<=0) return 0;
    FILE *f=fopen("position.csv","w");
    if (!f) return 0;
    fprintf(f,"t,x,e,J,P,I,D,u\n");
    double x=0,J=0,prev=target,first=0,second=0,last=0;
    for(int k=0;k<=count;k++) {
        double e=target-x;
        J+=e*dt;
        double P=kp*e,I=ki*J,D=kd*(e-prev)/dt,u=P+I+D;
        fprintf(f,"%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",k*dt,x,e,J,P,I,D,u);
        if(k==0) assert(D==0);
        if(k==1) first=x;
        if(k==2) second=x;
        last=x;
        x+=u*dt;
        prev=e;
    }
    fclose(f);
    printf("#229 count=%d x(0)=0.0000000000 x(3)=%.10f\n",count+1,last);
    printf("x(0.1)=%.10f x(0.2)=%.10f\n",first,second);
    assert(fabs(last-1.0857775541183516)<1e-12);
    return 1;
}

static int aw(const char *name,const double *errors,int count,double kp,double ki,double kd,double dt,double lo,double hi,int enabled,const double *expected) {
    if(!isfinite(dt)||dt<=0||lo>hi) return 0;
    char file[80];
    snprintf(file,sizeof(file),"%s.csv",name);
    FILE *f=fopen(file,"w");
    if(!f) return 0;
    fprintf(f,"n,e,I_before,I_try,P,D,u_un,u,I\n");
    double I=0,prev=0;
    printf("%s output=",name);
    for(int n=0;n<count;n++) {
        double e=errors[n],P=kp*e,I_try=I+ki*e*dt;
        double D=n==0?0:kd*(e-prev)/dt;
        double raw=P+I_try+D,u=fmax(lo,fmin(hi,raw)),old=I;
        if(u==raw || !enabled) I=I_try;
        fprintf(f,"%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",(double)n,e,old,I_try,P,D+0.0,raw,u,I);
        printf("%s%.4f",n?",":"",u);
        assert(fabs(u-expected[n])<1e-12);
        if(n==0) assert(D==0);
        if(ki==0) assert(I==0);
        prev=e;
    }
    printf("\n");
    fclose(f);
    return 1;
}

int main(void) {
    const double a[]={10,10,10,10}, b[]={10,10,10,-10}, edge[]={5,-5};
    const double low[]={-10,-10,10},prev[]={10,5,4},zero[]={10,10,-10};
    const double ea[]={20,30,40,50}, eb[]={15,15,15,-20}, en[]={15,15,15,10};
    const double ee[]={10,-5},el[]={-15,-15,20},ep[]={15,5,12},ez[]={10,10,-30};
    assert(!position(2,.5,.1,0,30,1));
    assert(!aw("invalid",a,4,1,1,0,-1,-100,100,1,ea));
    if(!position(2,.5,.1,.1,30,1)) return 1;
    if(!aw("A",a,4,1,1,0,1,-100,100,1,ea)) return 1;
    if(!aw("B",b,4,1,1,0,1,-100,15,1,eb)) return 1;
    if(!aw("no_aw",b,4,1,1,0,1,-100,15,0,en)) return 1;
    if(!aw("boundary",edge,2,1,1,0,1,-10,10,1,ee)) return 1;
    if(!aw("lower",low,3,1,1,0,1,-15,100,1,el)) return 1;
    if(!aw("prev",prev,3,1,1,1,1,-100,15,1,ep)) return 1;
    if(!aw("ki_zero",zero,3,1,0,1,1,-100,100,1,ez)) return 1;
    puts("checks=PASS");
    return 0;
}
