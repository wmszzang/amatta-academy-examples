#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int normalize(double *v, int n) {
    double length = 0;
    for (int i=0;i<n;i++) { if (!isfinite(v[i])) return 0; length=hypot(length,v[i]); }
    if (length == 0) return 0;
    for (int i=0;i<n;i++) v[i]/=length;
    return 1;
}

static int rodrigues(double *k, double theta, double R[3][3]) {
    if (!normalize(k,3) || !isfinite(theta)) return 0;
    double K[3][3]={{0,-k[2],k[1]},{k[2],0,-k[0]},{-k[1],k[0],0}};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        double squared=0;
        for (int t=0;t<3;t++) squared+=K[i][t]*K[t][j];
        R[i][j]=(i==j)+sin(theta)*K[i][j]+(1-cos(theta))*squared;
    }
    return 1;
}

static int quat_to_R(double *q, double R[3][3]) {
    if (!normalize(q,4)) return 0;
    double w=q[0],x=q[1],y=q[2],z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z); R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z); R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y); R[2][1]=2*(y*z+w*x); R[2][2]=1-2*(x*x+y*y);
    return 1;
}

static double determinant(double R[3][3]) {
    double d=0;
    for (int i=0;i<3;i++) d+=R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-R[1][(i+2)%3]*R[2][(i+1)%3]);
    return d;
}

static double orthogonal_error(double R[3][3]) {
    double error=0;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        double value=0;
        for (int t=0;t<3;t++) value+=R[t][i]*R[t][j];
        error=fmax(error,fabs(value-(i==j)));
    }
    return error;
}

static int R_to_quat(double R[3][3], double *q) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) if (!isfinite(R[i][j])) return 0;
    if (orthogonal_error(R)>1e-8 || fabs(determinant(R)-1)>1e-8) return 0;
    double t=R[0][0]+R[1][1]+R[2][2];
    if (t>0) {
        double s=2*sqrt(1+t);
        q[0]=s/4; q[1]=(R[2][1]-R[1][2])/s;
        q[2]=(R[0][2]-R[2][0])/s; q[3]=(R[1][0]-R[0][1])/s;
    } else {
        /* 동률일 때는 앞 성분을 선택한다. 최대 대각 분기 셋을 같은 순환식으로 계산한다. */
        int i=0;
        for (int d=1;d<3;d++) if (R[d][d]>R[i][i]) i=d;
        int j=(i+1)%3,h=(i+2)%3;
        double s=2*sqrt(fmax(0,1+R[i][i]-R[j][j]-R[h][h]));
        q[0]=(R[h][j]-R[j][h])/s; q[i+1]=s/4;
        q[j+1]=(R[i][j]+R[j][i])/s; q[h+1]=(R[i][h]+R[h][i])/s;
    }
    if (!normalize(q,4)) return 0;
    if (q[0]<0) for (int i=0;i<4;i++) q[i]=-q[i];
    return 1;
}

static void R_to_axis_angle(double R[3][3], double *q, double *a) {
    double length=hypot(hypot(q[1],q[2]),q[3]);
    double theta=acos(fmax(-1,fmin(1,(R[0][0]+R[1][1]+R[2][2]-1)/2)));
    if (length<1e-12) { for (int i=0;i<5;i++) a[i]=0; return; }
    if (fabs(sin(theta))<1e-6) {
        theta=2*atan2(length,q[0]);
        for (int i=0;i<3;i++) a[i]=q[i+1]/length;
    } else {
        a[0]=(R[2][1]-R[1][2])/(2*sin(theta));
        a[1]=(R[0][2]-R[2][0])/(2*sin(theta));
        a[2]=(R[1][0]-R[0][1])/(2*sin(theta));
    }
    a[3]=theta; a[4]=1;
}

static void write_row(FILE *f, double *values, int n) {
    for (int i=0;i<n;i++) fprintf(f,"%s%.4f",i?",":"",fabs(values[i])<0.00005?0:values[i]);
    fprintf(f,"\n");
}

int main(int argc, char **argv) {
    const char *mode=argc>1?argv[1]:"axis";
    double input[9], R[3][3],q[4],axis[5];
    int n=strcmp(mode,"matrix")==0?9:4;
    for (int i=0;i<n;i++) if (scanf("%lf",&input[i])!=1) { fprintf(stderr,"INPUT ERROR: wrong input count\n"); return 2; }
    char extra;
    if (scanf(" %c",&extra)==1) { fprintf(stderr,"INPUT ERROR: extra input\n"); return 2; }
    int ok=1;
    if (strcmp(mode,"axis")==0) ok=rodrigues(input,input[3],R);
    else if (strcmp(mode,"quat")==0) ok=quat_to_R(input,R);
    else if (strcmp(mode,"matrix")==0) for (int i=0;i<3;i++) for (int j=0;j<3;j++) R[i][j]=input[3*i+j];
    else ok=0;
    if (!ok || !R_to_quat(R,q)) { fprintf(stderr,"INPUT ERROR: invalid rotation input\n"); return 2; }
    R_to_axis_angle(R,q,axis);
    FILE *f=fopen("R.csv","w");
    if (!f) return 3;
    puts("R:");
    for (int i=0;i<3;i++) { write_row(f,R[i],3); write_row(stdout,R[i],3); }
    fclose(f);
    f=fopen("quat.csv","w"); if (!f) return 3;
    write_row(f,q,4); fclose(f); puts("quat:"); write_row(stdout,q,4);
    f=fopen("axis_angle.csv","w"); if (!f) return 3;
    write_row(f,axis,5); fclose(f); puts("axis_angle:"); write_row(stdout,axis,5);
    printf("det=%.12f; orthogonal_error=%.3e\n",determinant(R),orthogonal_error(R));
    return 0;
}
