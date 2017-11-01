#!/usr/bin/env python

import geometry3d as g3d
from math import copysign


def intersect(bbox, orig, invdir, sign):
    tmin = (bbox[sign[0]][0] - orig[0]) * invdir[0]
    tmax = (bbox[1-sign[0]][0] - orig[0]) * invdir[0]
    tymin = (bbox[sign[1]][1] - orig[1]) * invdir[1]
    tymax = (bbox[1-sign[1]][1] - orig[1]) * invdir[1]

    if(tmin > tymax) or (tymin > tmax):
        return False
    if tymin > tmin:
        tmin = tymin
    if tymax < tmax:
        tmax = tymax

    tzmin = (bbox[sign[2]][2] - orig[2]) * invdir[2]
    tzmax = (bbox[1-sign[2]][2] - orig[2]) * invdir[2]

    if (tmin > tzmax) or (tzmin > tmax):
        return False
    if tzmin > tmin:
        tmin = tzmin
    if tzmax < tmax:
        tmax = tzmax

    return True
    #return (tmin < t1) and (tmax > t0)


s = 150.0
bb = [ [s + 0.0, 0.0, 0.0], [s + 100.0, 100.0, 100.0] ]
origin = [200.0, 50.0, 50.0]
dir = [-1.0, 0.0, 0.0]
#bb = [[1250.0, 0.0, 875.0], [1875.0, 1010.0, 1750.0]]
#origin = [1000.0, 300.0, 1000.0]

#res = g3d.boxrayintersect(bb, origin, dir)
inv_dir = [1.0 / (x if abs(x) > 0.00001 else copysign(0.00001, x)) for x in dir]
sign = [int(inv_dir[0] < 0), int(inv_dir[1] < 0), int(inv_dir[2] < 0)]
res = intersect(bb, origin, inv_dir, sign)
print(res)
res = g3d.boxrayintersectBL(bb, origin, inv_dir)
print(res)
