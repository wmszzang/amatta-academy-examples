import math

def diff_ik(v,w,r,track):
    left,right=v-w*track/2,v+w*track/2
    return left,right,left/r,right/r

def mecanum_ik(vx,vy,w,r,lx,ly):
    k=lx+ly
    return ((vx-vy-k*w)/r,(vx+vy+k*w)/r,(vx+vy-k*w)/r,(vx-vy+k*w)/r)

def mecanum_fk(wheels,r,lx,ly):
    a,b,c,d=wheels
    return r*(a+b+c+d)/4,r*(-a+b+c-d)/4,r*(-a+b-c+d)/(4*(lx+ly))

def odom(dl,dr,track,n,midpoint=True):
    x=y=theta=0.0; out=[(x,y,theta)]
    dc,dtheta=(dl+dr)/2,(dr-dl)/track
    for _ in range(n):
        direction=theta+dtheta/2 if midpoint else theta
        x+=dc*math.cos(direction); y+=dc*math.sin(direction); theta+=dtheta
        out.append((x,y,theta))
    return out

def pure_pursuit(goal,wheelbase):
    kappa=2*goal[1]/sum(x*x for x in goal)
    return kappa,math.atan(wheelbase*kappa)

def ackermann(wheelbase,track,radius):
    return math.atan(wheelbase/(radius-track/2)),math.atan(wheelbase/(radius+track/2))

def bicycle_step(p,v,delta,wheelbase,dt):
    x,y,theta=p
    return x+v*math.cos(theta)*dt,y+v*math.sin(theta)*dt,theta+v*math.tan(delta)*dt/wheelbase
