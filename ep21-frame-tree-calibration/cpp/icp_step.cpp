#include <cmath>
#include <cstdio>
// 2x2 대칭행렬 H^T H는 한 번의 Jacobi 회전으로 대각화한다.
bool estimate_rigid_2d(const double Q[][2],const double P[][2],int n,double R[2][2],double t[2]){
    double cq[2]={0,0},cp[2]={0,0},H[2][2]={{0,0},{0,0}};
    if(n<2)return false;
    for(int i=0;i<n;i++)for(int j=0;j<2;j++){cq[j]+=Q[i][j]/n;cp[j]+=P[i][j]/n;}
    for(int i=0;i<n;i++)for(int j=0;j<2;j++)for(int k=0;k<2;k++)H[j][k]+=(Q[i][j]-cq[j])*(P[i][k]-cp[k]);
    double a=H[0][0]*H[0][0]+H[1][0]*H[1][0], b=H[0][0]*H[0][1]+H[1][0]*H[1][1], d=H[0][1]*H[0][1]+H[1][1]*H[1][1];
    double phi=.5*std::atan2(2*b,a-d),c=std::cos(phi),s=std::sin(phi);
    double V[2][2]={{c,-s},{s,c}},U[2][2];
    for(int j=0;j<2;j++){
        for(int i=0;i<2;i++)U[i][j]=H[i][0]*V[0][j]+H[i][1]*V[1][j];
        double sigma=std::hypot(U[0][j],U[1][j]);
        if(sigma<1e-12)return false;
        U[0][j]/=sigma;U[1][j]/=sigma;
    }
    double detV=V[0][0]*V[1][1]-V[0][1]*V[1][0],detU=U[0][0]*U[1][1]-U[0][1]*U[1][0];
    if(detV*detU<0){V[0][1]*=-1;V[1][1]*=-1;}
    for(int i=0;i<2;i++){for(int j=0;j<2;j++)R[i][j]=V[i][0]*U[j][0]+V[i][1]*U[j][1];t[i]=cp[i]-R[i][0]*cq[0]-R[i][1]*cq[1];}
    return true;
}
int main(){
    const double Q[4][2]={{0,0},{1,0},{1,1},{0,1}},P[4][2]={{2,3},{2,4},{1,4},{1,3}};double R[2][2],t[2];
    if(!estimate_rigid_2d(Q,P,4,R,t))return 2;
    std::printf("angle=%.4f deg, t=(%.4f, %.4f)\n",std::atan2(R[1][0],R[0][0])*180/std::acos(-1),t[0],t[1]);return 0;
}
