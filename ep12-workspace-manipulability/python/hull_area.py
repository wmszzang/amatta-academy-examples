"""#497 볼록껍질과 #287 신발끈 공식."""
def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])

def convex_hull(points):
    points = sorted(set(map(tuple, points)))
    if len(points) <= 2:
        return points
    def chain(sequence):
        result = []
        for p in sequence:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result
    return chain(points)[:-1] + chain(reversed(points))[:-1]

def shoelace(v):
    return abs(sum(v[i][0]*v[(i+1)%len(v)][1]-v[(i+1)%len(v)][0]*v[i][1]
                   for i in range(len(v))))/2 if len(v) >= 3 else 0.0

if __name__ == '__main__':
    print(convex_hull([(0,0),(1,1),(2,2),(2,0),(0,2),(1,0)]))
    print(shoelace([(0,0),(4,0),(4,3),(2,5),(0,3)]))
