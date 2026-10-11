/* EP.29: 반환한 배열은 호출자가 free 한다. 종료점 가속도는 #119 표기이다. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct { double t, x, v, a; } Sample;
typedef struct { double x, y; } Point;

Sample *trapezoidal_profile(double distance, double max_speed, double acceleration, double dt, size_t *count) {
    *count = 0;
    if (distance <= 0 || max_speed <= 0 || acceleration <= 0 || dt <= 0) return NULL;
    double peak = fmin(max_speed, sqrt(distance * acceleration));
    double ta = peak / acceleration, da = .5 * acceleration * ta * ta;
    double cruise = fmax(0., (distance - 2 * da) / peak);
    double t2 = ta + cruise, total = 2 * ta + cruise;
    size_t intervals = (size_t)ceil(total / dt);
    Sample *rows = malloc((intervals + 1) * sizeof(*rows));
    if (!rows) return NULL;
    for (size_t i = 0; i <= intervals; ++i) {
        double t = i == intervals ? total : i * dt;
        if (i < intervals && t >= total - 1e-12) continue;
        Sample row = {t, 0, 0, 0};
        if (t < ta) { row.x=.5*acceleration*t*t; row.v=acceleration*t; row.a=acceleration; }
        else if (t < t2) { row.x=da+peak*(t-ta); row.v=peak; row.a=0; }
        else { double remaining=total-t; row.x=distance-.5*acceleration*remaining*remaining; row.v=acceleration*remaining; row.a=-acceleration; }
        rows[(*count)++] = row;
    }
    return rows;
}

Point *transform_points(const Point *points, size_t count, double degrees, double tx, double ty) {
    Point *result = malloc(count * sizeof(*result));
    if (!result) return NULL;
    double angle = degrees * acos(-1.) / 180., c = cos(angle), s = sin(angle);
    for (size_t i=0; i<count; ++i) {
        result[i].x = c*points[i].x-s*points[i].y+tx;
        result[i].y = s*points[i].x+c*points[i].y+ty;
    }
    return result;
}

int main(int argc, char **argv) {
    double distance=argc>1?atof(argv[1]):10., dt=argc>2?atof(argv[2]):.25;
    size_t count;
    Sample *rows=trapezoidal_profile(distance,2.,1.,dt,&count);
    Point original[4]={{0,0},{1,0},{1,1},{0,1}};
    Point *points=transform_points(original,4,30.,1.,.5);
    if (!rows || !points) { free(rows); free(points); return 1; }
    FILE *trajectory=fopen("trajectory.csv","w"), *vertices=fopen("points.csv","w");
    if (!trajectory || !vertices) { if(trajectory) fclose(trajectory); if(vertices) fclose(vertices); free(rows); free(points); return 1; }
    fprintf(trajectory,"t,x,v,a\n");
    for(size_t i=0;i<count;++i) fprintf(trajectory,"%.10f,%.10f,%.10f,%.10f\n",rows[i].t,rows[i].x,rows[i].v,rows[i].a);
    fprintf(vertices,"id,x,y,xp,yp\n");
    for(size_t i=0;i<4;++i) fprintf(vertices,"%c,%.10f,%.10f,%.10f,%.10f\n",(int)('A'+i),original[i].x,original[i].y,points[i].x,points[i].y);
    Sample last=rows[count-1];
    printf("rows=%zu T=%.10f x=%.10f v=%.10f a=%.10f\n",count,last.t,last.x,last.v,last.a);
    for(size_t i=0;i<4;++i) printf("%c %.10f %.10f\n",(int)('A'+i),points[i].x,points[i].y);
    fclose(trajectory); fclose(vertices); free(rows); free(points);
    return 0;
}
