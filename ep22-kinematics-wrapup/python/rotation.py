import math

def rotation_zyx(roll,pitch,yaw):
    a,b,c=roll,pitch,yaw
    ca,sa,cb,sb,cc,sc=math.cos(a),math.sin(a),math.cos(b),math.sin(b),math.cos(c),math.sin(c)
    return ((cc*cb,cc*sb*sa-sc*ca,cc*sb*ca+sc*sa),
            (sc*cb,sc*sb*sa+cc*ca,sc*sb*ca-cc*sa),(-sb,cb*sa,cb*ca))

def euler_zyx(r):
    cp=math.hypot(r[0][0],r[1][0]); pitch=math.atan2(-r[2][0],cp)
    if cp<1e-9: return math.atan2(-r[1][2],r[1][1]),pitch,0.0
    return math.atan2(r[2][1],r[2][2]),pitch,math.atan2(r[1][0],r[0][0])
