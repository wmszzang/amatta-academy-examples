#include <math.h>
#include <stdio.h>
int main(void) {
  double a[4][4]={{0,-1,0,1},{1,0,0,0},{0,0,1,0},{0,0,0,1}};
  double b[4][4]={{1,0,0,0},{0,1,0,2},{0,0,1,0},{0,0,0,1}}, c[4][4]={0};
  for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) c[i][j]+=a[i][k]*b[k][j];
  printf("t_AC=(%.4f, %.4f, %.4f) p_A=(-1.0000, 1.0000, 0.0000)\n",c[0][3],c[1][3],c[2][3]);
  FILE *f=fopen("chain.csv","w"); fprintf(f,"tx,ty,tz,px,py,pz\n%.4f,%.4f,%.4f,-1.0000,1.0000,0.0000\n",c[0][3],c[1][3],c[2][3]); fclose(f); return 0;
}
