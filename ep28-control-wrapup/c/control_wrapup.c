#include <stdio.h>
#include <math.h>
#include <assert.h>

static void limited(const char *name, const double *errors, int count, double upper, int hold) {
    char filename[64];
    snprintf(filename,sizeof(filename),"%s.csv",name);
    FILE *f=fopen(filename,"wb"); assert(f);
    fprintf(f,"step,error,i_before,i_trial,raw,command,i_after\n");
    double integral=0.,prev=0.,before=0.,trial=0.,raw=0.,command=0.;
    for(int n=0;n<count;n++) {
        double error=errors[n];
        before=integral;
        trial=integral+1.*error*1.;
        double derivative=n==0?0.:(error-prev)/1.;
        raw=1.*error+trial+0.*derivative;
        command=fmax(-100.,fmin(upper,raw));
        if(!hold || command==raw) integral=trial;
        fprintf(f,"%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",(double)n+1,error,before,trial,raw,command,integral);
        prev=error;
    }
    fclose(f);
    printf("%s: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",name,(double)count,errors[count-1],before,trial,raw,command,integral);
}

int main(void) {
    FILE *f=fopen("plant.csv","wb"); assert(f);
    fprintf(f,"t_start,y_before,command,t_end,y_after\n");
    double y=0.,integral=0.,prev=0.,before=0.,command=0.;
    for(int k=0;k<200;k++) {
        before=y;
        double error=1.-before;
        integral+=error*.05;
        double derivative=(error-prev)/.05;
        command=2.*error+1.*integral+.1*derivative;
        y=before+.05*(1.*command-before)/1.;
        fprintf(f,"%.9f,%.9f,%.9f,%.9f,%.9f\n",k*.05,before,command,(k+1)*.05,y);
        if(k==0) { assert(fabs(.1*derivative-2.)<1e-12); assert(fabs(y-.2025)<1e-12); }
        prev=error;
    }
    fclose(f); assert(fabs(y-.9939577634244283)<1e-12);
    printf("plant: %.6f, %.6f, %.6f, %.6f, %.6f\n",9.95,before,command,10.,y);
    const double normal[]={10.,10.,10.,10.}, sat[]={10.,10.,10.,-10.}, boundary[]={7.5};
    limited("normal",normal,4,100.,1); limited("hold",sat,4,15.,1);
    limited("no_hold",sat,4,15.,0); limited("boundary",boundary,1,15.,1);
    f=fopen("kalman.csv","wb"); assert(f);
    fprintf(f,"step,predicted,predicted_var,weight,residual,position,variance\n");
    double position=0.,variance=1.,predicted=0.,predicted_var=0.,weight=0.,residual=0.;
    const double measurements[]={1.2,2.1};
    for(int k=0;k<2;k++) {
        predicted=position+1.; predicted_var=variance+.1;
        weight=predicted_var/(predicted_var+.5); residual=measurements[k]-predicted;
        position=predicted+weight*residual; variance=(1.-weight)*predicted_var;
        fprintf(f,"%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",(double)k+1,predicted,predicted_var,weight,residual,position,variance);
    }
    fclose(f); assert(fabs(position-3201./1510.)<1e-12); assert(fabs(variance-71./302.)<1e-12);
    printf("kalman: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",2.,predicted,predicted_var,weight,residual,position,variance);
    f=fopen("complementary.csv","wb"); assert(f);
    fprintf(f,"step,time,predicted,measured,angle,gyro_only\n");
    double angle=0.;
    for(int k=1;k<=20;k++) {
        double time=k*.1,measured=10.*time;
        predicted=angle+12.*.1;
        angle=.95*predicted+(1.-.95)*measured;
        fprintf(f,"%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",(double)k,time,predicted,measured,angle,12.*time);
        assert(fabs(angle-(k+3.8*(1.-pow(.95,k))))<1e-12);
    }
    fclose(f);
    printf("complementary: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",20.,2.,predicted,20.,angle,24.);
    return 0;
}
