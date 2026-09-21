#include <math.h>
#include <stdio.h>
#include <string.h>

/* 행 우선 9성분이며 내부 계산은 반올림하지 않는다. */
void euler_to_R(double roll, double pitch, double yaw, double R[9]) {
    double cr=cos(roll), sr=sin(roll), cp=cos(pitch), sp=sin(pitch);
    double cy=cos(yaw), sy=sin(yaw);
    R[0]=cy*cp; R[1]=cy*sp*sr-sy*cr; R[2]=cy*sp*cr+sy*sr;
    R[3]=sy*cp; R[4]=sy*sp*sr+cy*cr; R[5]=sy*sp*cr-cy*sr;
    R[6]=-sp; R[7]=cp*sr; R[8]=cp*cr;
}

void rot_to_euler_zyx(const double R[9], double *roll, double *pitch, double *yaw) {
    *pitch=atan2(-R[6],hypot(R[0],R[3]));
    if (fabs(cos(*pitch))<1e-6) {
        *yaw=0.0;
        *roll=atan2(-R[5],R[4]);
    } else {
        *roll=atan2(R[7],R[8]);
        *yaw=atan2(R[3],R[0]);
    }
}

double roundtrip_err(const double R[9]) {
    double roll,pitch,yaw,Q[9],err=0.0;
    rot_to_euler_zyx(R,&roll,&pitch,&yaw);
    euler_to_R(roll,pitch,yaw,Q);
    for(int i=0;i<9;i++) err=fmax(err,fabs(Q[i]-R[i]));
    return err;
}

double heading_from_gyro(const double *omega,int n,double dt,double bias,double theta0) {
    double theta=theta0;
    for(int i=0;i<n-1;i++) theta+=0.5*((omega[i]-bias)+(omega[i+1]-bias))*dt;
    return theta;
}

int main(int argc,char **argv) {
    const char *names[]={"normal","plus90","minus90"};
    double pi=acos(-1.0), angles[3][3]={{.3,-.5,1.2},{.2,0,.7},{.2,0,.7}};
    double matrices[3][9];
    int count=3;
    angles[1][1]=pi/2; angles[2][1]=-pi/2;
    for(int k=0;k<3;k++) euler_to_R(angles[k][0],angles[k][1],angles[k][2],matrices[k]);
    if(argc>1 && strcmp(argv[1],"--stdin")==0) {
        for(int i=0;i<9;i++) if(scanf("%lf",&matrices[0][i])!=1 || !isfinite(matrices[0][i])) return 2;
        double extra;
        if(scanf("%lf",&extra)==1) return 2;
        count=1; names[0]="input";
    }
    FILE *f=fopen("euler.csv","w"), *g=fopen("roundtrip.csv","w");
    if(!f || !g) return 3;
    fprintf(f,"case,roll,pitch,yaw\n"); fprintf(g,"case,max_error\n");
    for(int k=0;k<count;k++) {
        double roll,pitch,yaw,err=roundtrip_err(matrices[k]);
        rot_to_euler_zyx(matrices[k],&roll,&pitch,&yaw);
        fprintf(f,"%s,%.4f,%.4f,%.4f\n",names[k],roll,pitch,yaw);
        fprintf(g,"%s,%.12e\n",names[k],err);
        printf("%s %.4f %.4f %.4f error=%.12e\n",names[k],roll,pitch,yaw,err);
    }
    fclose(f); fclose(g);
    double omega[]={.1,.2,.2,.1};
    double heading=heading_from_gyro(omega,4,.5,.1,0);
    f=fopen("heading.txt","w"); if(!f) return 3;
    fprintf(f,"%.4f\n",heading); fclose(f);
    printf("heading=%.4f\n",heading);
    return 0;
}
