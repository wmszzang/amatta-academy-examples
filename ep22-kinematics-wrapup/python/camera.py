def project(point,f,cx,cy):
    x,y,z=point
    if z<=0: raise ValueError('INVALID_DEPTH')
    return cx+f*x/z,cy+f*y/z

def backproject(ul,vl,ur,f,baseline,cx,cy):
    disparity=ul-ur
    if disparity<=0: raise ValueError('INVALID_DISPARITY')
    z=f*baseline/disparity
    return (ul-cx)*z/f,(vl-cy)*z/f,z
