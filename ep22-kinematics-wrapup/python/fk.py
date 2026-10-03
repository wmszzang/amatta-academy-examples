import math

def fk2(q, lengths):
    a, b = q; l1, l2 = lengths
    return l1*math.cos(a)+l2*math.cos(a+b), l1*math.sin(a)+l2*math.sin(a+b)

def dh_planar(angle, length):
    c,s=math.cos(angle),math.sin(angle)
    return ((c,-s,length*c),(s,c,length*s),(0,0,1))

def matmul(a,b):
    return tuple(tuple(sum(x*y for x,y in zip(row,col)) for col in zip(*b)) for row in a)
