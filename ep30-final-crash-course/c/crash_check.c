/* EP.30 기존 입력의 계산과 독립 기대값 비교. 표시할 때만 반올림한다. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void emit(FILE *file, const char *name, int n, const double *values, const double *expected) {
    int i;
    for(i=0;i<n;i++) if(fabs(values[i]-expected[i])>1e-8) {
        fprintf(stderr,"FAIL: %s value %d\n",name,i); exit(1);
    }
    fprintf(file,"%s",name); printf("%s",name);
    for(i=0;i<n;i++) {fprintf(file,",%.10f",values[i]);printf(",%.10f",values[i]);}
    fprintf(file,",PASS\n");printf(",PASS\n");
}

int main(void) {
    const double pi=acos(-1.0);
    double v[4],e[4],c=cos(pi/6),s=sin(pi/6),a,b,c2,s2,x,p,k,pp;
    int i,j;
    const double square[4][2]={{0,0},{1,0},{1,1},{0,1}};
    const double expected[4][2]={{1,.5},{1.8660254037844386,1},{1.3660254037844386,1.8660254037844386},{.5,1.3660254037844386}};
    const int grid[4][4]={{0,0,0,0},{1,1,0,1},{0,0,0,0},{0,1,1,0}};
    const int dr[4]={0,1,0,-1},dc[4]={1,0,-1,0};
    int queue[16],prev[16],head=0,tail=0,cur,count=0;
    char name[64];
    FILE *file=fopen("crash_check.csv","w");
    if(!file){perror("crash_check.csv");return 1;}
    v[0]=2.0/1.0; v[1]=(10.-2.*2./1.)/2.;v[2]=2*v[0]+v[1];v[3]=2*(v[0]+v[1]);
    e[0]=2;e[1]=3;e[2]=7;e[3]=10;emit(file,"motion_119",4,v,e);
    for(i=0;i<4;i++) {
        v[0]=square[i][0]*c-square[i][1]*s+1;v[1]=square[i][0]*s+square[i][1]*c+.5;
        snprintf(name,sizeof(name),"transform_226_%d",i);emit(file,name,2,v,expected[i]);
    }
    a=25*pi/180;b=50*pi/180;
    v[0]=1.4*cos(a)+.9*cos(a+b);v[1]=1.4*sin(a)+.9*sin(a+b);
    e[0]=1.5017680424435782;e[1]=1.4609988100971407;emit(file,"fk_ep09",2,v,e);
    c2=(1.5*1.5+1-2)/2;s2=sqrt(1-c2*c2);b=atan2(s2,c2);a=atan2(1,1.5)-atan2(s2,1+c2);
    v[0]=a*180/pi;v[1]=b*180/pi;v[2]=cos(a)+cos(a+b);v[3]=sin(a)+sin(a+b);
    e[0]=8.031161252724512;e[1]=51.31781254651057;e[2]=1.5;e[3]=1;emit(file,"ik_222",4,v,e);
    v[0]=(-2*sin(.5)-1.5*sin(1))*.4+(-1.5*sin(1))*(-.3);
    v[1]=(2*cos(.5)+1.5*cos(1))*.4+1.5*cos(1)*(-.3);
    e[0]=-.509761078604546;e[1]=.7831113953925192;emit(file,"jacobian_239",2,v,e);
    for(i=0;i<16;i++)prev[i]=-2;
    queue[tail++]=0;prev[0]=-1;
    while(head<tail) {
        cur=queue[head++];if(cur==15)break;
        for(j=0;j<4;j++) {
            int r=cur/4+dr[j],col=cur%4+dc[j],next=r*4+col;
            if(r>=0&&r<4&&col>=0&&col<4&&!grid[r][col]&&prev[next]==-2) {
                prev[next]=cur;queue[tail++]=next;
            }
        }
    }
    for(cur=15;cur!=-1;cur=prev[cur]){if(cur<0){fclose(file);return 1;}count++;}
    v[0]=count;v[1]=count-1;v[2]=6;e[0]=7;e[1]=6;e[2]=6;emit(file,"path_241",3,v,e);
    x=0;p=1;
    for(i=0;i<2;i++) {
        const double z=i==0?1.2:2.1;x+=1;pp=p+.1;k=pp/(pp+.5);x+=k*(z-x);p=(1-k)*pp;
        v[0]=x;v[1]=p;e[0]=i==0?1.1375:2.119867549668874;e[1]=i==0?.34375:.23509933774834438;
        snprintf(name,sizeof(name),"kalman_399_%d",i+1);emit(file,name,2,v,e);
    }
    v[0]=sqrt(1);v[1]=2*sqrt(1);e[0]=1;e[1]=2;emit(file,"triangle_ep29",2,v,e);
    v[0]=(4.-2.*2./1.)/2.;v[1]=4;e[0]=0;e[1]=4;emit(file,"boundary_ep29",2,v,e);
    v[0]=10-.5*pow(7-6.75,2);v[1]=7-6.75;e[0]=9.96875;e[1]=.25;emit(file,"endpoint_119",2,v,e);
    v[0]=fmod(6+pi,2*pi)-pi;e[0]=-.28318530717958623;emit(file,"wrap_ep13",1,v,e);
    if(fclose(file)!=0)return 1;
    return 0;
}
