
from math import sqrt, sin, cos, inf


# Vector arithmetic functions
def plus(u, v):
    return [uu + vv for uu, vv in zip(u, v)]

def minus(u, v):
    return [uu - vv for uu, vv in zip(u, v)]

def mul_scalar(v, s):
    return [vv*s for vv in v]

def dot(u, v):
    return sum(uu * vv for uu, vv in zip(u, v))

def cross(u, v):
    return [ u[1]*v[2] - u[2]*v[1], 
             u[2]*v[0] - u[0]*v[2], 
             u[0]*v[1] - u[1]*v[0] ]

def length(v):
    return sqrt(dot(v, v))

def normalize(v):
    m = length(v)
    if m == 0:
        print('*** Zero length ***')
        print(v)
        return v
    return [vv/m for vv in v]


# Calculates 3D bounding box of the mesh.
# triangles assumed to be a flat array with every three elements
# representing the vertex.
# Returns tuple (max_coord, min_coord) where max and min coords have three
# elements representing the x, y and z coordinates of the box's corners.
def bounding_box(triangles):
    max_coord = triangles[0:3]
    min_coord = triangles[0:3]
    for i in range(0, len(triangles), 3):
        for n, v in enumerate(triangles[i:i+3]):
            if v > max_coord[n]:
                max_coord[n] = v
            elif v < min_coord[n]:
                min_coord[n] = v
    return (min_coord, max_coord)


def rotateY(v, radians):
    return [
        v[0] * cos(radians) - v[2] * sin(radians),
        v[1],
        v[0] * sin(radians) + v[2] * cos(radians),
    ]


# Branchless intersections between axis-aligned bounding (AABB) box and ray.
# 
# For more details:
# https://tavianator.com/fast-branchless-raybounding-box-intersections-part-2-nans/
#
def boxrayintersectBL(bbox, origin, dir_inv):
    t1 = (bbox[0][0] - origin[0]) * dir_inv[0]
    t2 = (bbox[1][0] - origin[0]) * dir_inv[0]
 
    tmin = min(t1, t2)
    tmax = max(t1, t2)
 
    t1 = (bbox[0][1] - origin[1]) * dir_inv[1]
    t2 = (bbox[1][1] - origin[1]) * dir_inv[1]

    tmin = max(tmin, min(t1, t2))
    tmax = min(tmax, max(t1, t2))
 
    t1 = (bbox[0][2] - origin[2]) * dir_inv[2]
    t2 = (bbox[1][2] - origin[2]) * dir_inv[2]

    tmin = max(tmin, min(t1, t2))
    tmax = min(tmax, max(t1, t2))

    return tmax > max(tmin, 0.0)


# https://github.com/stackgl/ray-aabb-intersection
# ro - origin
# rd - direction
def boxrayintersect(aabb, ro, rd):
    d = distance(ro, rd, aabb)
    if d == inf:
        out = None
    else:
        out = [0]*len(ro)
        for i in range(len(ro)):
            out[i] = ro[i] + rd[i] * d

    return out


def distance(ro, rd, aabb):
    dims = len(ro)
    lo = -inf
    hi =  inf

    for i in range(dims):
        dimLo = (aabb[0][i] - ro[i]) / rd[i]
        dimHi = (aabb[1][i] - ro[i]) / rd[i]

        if dimLo > dimHi:
            tmp = dimLo
            dimLo = dimHi
            dimHi = tmp

        if dimHi < lo or dimLo > hi:
            return inf

        if dimLo > lo:
            lo = dimLo
        if dimHi < hi:
            hi = dimHi

    if lo > hi:
        return inf

    return lo


# Moeller-Trumbore ray-triangle intersection algorithm
# For more details:
# https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
# and
# http://webserver2.tecgraf.puc-rio.br/~mgattass/cg/trbRR/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
#
# Parameters:
#
# orig - tuple with x,y,z coordinates of the ray origin
#
# dir - unit direction vector of the ray (x,y,z tuple)
#
# vert - triangle to test as tuple (vertex0, vertex1, vertex2, edge1, edge2)
# where each vertex is a tuple of (x,y,z) vertex coordinates and edge1 =
# vert[1] - vert[0], and edge2 = vert[2] - vert[0]
#
# test_cull - if True, triangles are considered single sided. If ray
# intersects triangle from "opposite" side, it is not considered as
# intersection. If False, triangles are considered double-sided.
#
# Returns:
# tuple I, (t, u, v) where I is the Boolean value indicating whether the ray
# intersects the triangle. t is the distance to the intersection. u and v
# are coordinates of the intersection.
def intersect_triangle(orig, dir, vert, test_cull=True):
    # To ensure numerical stability the test which eliminates parallel rays
    # must compare the determinant to a small interval around zero.
    epsilon = 1e-5
    # Find vectors for two edges sharing vert0
    #edge1 = minus(vert[1], vert[0])
    #edge2 = minus(vert[2], vert[0])
    edge1 = vert[3]
    edge2 = vert[4]
    # Begin calculating determinant - also used to calculate U parameter
    pvec = cross(dir, edge2)
    # If determinant is near zero, ray lies in plane of triangle
    det = dot(edge1, pvec)

    if test_cull:
        if det < epsilon:
            return False, (0,0,0)
        # Calculate distance from vert0 to ray origin
        tvec = minus(orig, vert[0])
        # Calculate U parameter and test bounds
        u = dot(tvec, pvec)
        if u < 0 or u > det:
            return False, (0,0,0)
        # Prepare to test V parameter
        qvec = cross(tvec, edge1)
        # Calculate V parameter and test bounds
        v = dot(dir, qvec)
        if v < 0 or u + v > det:
            return False, (0,0,0)
        # Calculate t, scale parameters, ray intersects triangle
        t = dot(edge2, qvec)
        inv_det = 1.0 / det
        t *= inv_det
        u *= inv_det
        v *= inv_det
    else: # The non-culling branch
        if det > -epsilon and det < epsilon:
            return False, (0,0,0)
        inv_det = 1.0 / det
        # Calculate distance from vert0 to ray origin
        tvec = minus(orig, vert[0])
        # Calculate U parameter and test bounds
        u = dot(tvec, pvec) * inv_det;
        if u < 0 or u > 1.0:
            return False, (0,0,0)
        # Prepare to test V parameter
        qvec = cross(tvec, edge1)
        # Calculate V parameter and test bounds
        v = dot(dir, qvec) * inv_det
        if v < 0 or u + v > 1.0:
            return False, (0,0,0)
        # Calculate t, ray intersects triangle
        t = dot(edge2, qvec) * inv_det

    return True, (t, u, v)


# AABB-triangle overlap test code
# by Tomas Akenine-Möller
#
# Function: int triBoxOverlap(float boxcenter[3],
#          float boxhalfsize[3],float triverts[3][3]);
#
# For more details:
# http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
#
def findminmax(x0, x1, x2):
    minv = maxv = x0
    if x1 < minv:
        minv = x1
    if x1 > maxv:
        maxv = x1
    if x2 < minv:
        minv = x2
    if x2 > maxv:
        maxv = x2

    return minv, maxv


def planeboxoverlap(normal, vert, maxbox):
    vmin = [0] * 3
    vmax = [0] * 3
    for q,v in enumerate(vert):
        if normal[q] > 0.0:
            vmin[q] = -maxbox[q] - v
            vmax[q] =  maxbox[q] - v
        else:
            vmin[q] =  maxbox[q] - v
            vmax[q] = -maxbox[q] - v

    if dot(normal, vmin) > 0.0:
        return False

    if dot(normal, vmax) >= 0.0:
        return True

    return False


#======================== X-tests ========================

def AXISTEST_X01(a, b, fa, fb, v0, v1, v2, boxhalfsize):
    p0 = a*v0[1] - b*v0[2]
    p2 = a*v2[1] - b*v2[2]
    if p0 < p2:
        minv = p0
        maxv = p2
    else:
        minv = p2
        maxv = p0
    rad = fa * boxhalfsize[1] + fb * boxhalfsize[2]
    if minv > rad or maxv < -rad:
        return False
    return True

def AXISTEST_X2(a, b, fa, fb, v0, v1, v2, boxhalfsize):
    p0 = a*v0[1] - b*v0[2]
    p1 = a*v1[1] - b*v1[2]
    if p0 < p1:
        minv = p0
        maxv = p1
    else:
        minv = p1
        maxv = p0
    rad = fa * boxhalfsize[1] + fb * boxhalfsize[2]
    if minv > rad or maxv < -rad:
        return False
    return True

#======================== Y-tests ========================

def AXISTEST_Y02(a, b, fa, fb, v0, v1, v2, boxhalfsize):
    p0 = -a*v0[0] + b*v0[2]
    p2 = -a*v2[0] + b*v2[2]
    if p0 < p2:
        minv = p0
        maxv = p2
    else:
        minv = p2
        maxv = p0
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[2]
    if minv > rad or maxv < -rad:
        return False
    return True

def AXISTEST_Y1(a, b, fa, fb, v0, v1, v2, boxhalfsize):
    p0 = -a*v0[0] + b*v0[2]
    p1 = -a*v1[0] + b*v1[2]
    if p0 < p1:
        minv = p0
        maxv = p1
    else:
        minv = p1
        maxv = p0
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[2]
    if minv > rad or maxv < -rad:
        return False
    return True

#======================== Z-tests ========================

def AXISTEST_Z12(a, b, fa, fb, v0, v1, v2, boxhalfsize):
    p1 = a*v1[0] - b*v1[1]
    p2 = a*v2[0] - b*v2[1]
    if p2 < p1:
        minv = p2
        maxv = p1
    else:
        minv = p1
        maxv = p2
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[1]
    if minv > rad or maxv < -rad:
        return False
    return True

def AXISTEST_Z0(a, b, fa, fb, v0, v1, v2, boxhalfsize):
    p0 = a*v0[0] - b*v0[1]
    p1 = a*v1[0] - b*v1[1]
    if p0 < p1:
        minv = p0
        maxv = p1
    else:
        minv = p1
        maxv = p0
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[1]
    if minv > rad or maxv < -rad:
        return False
    return True


# Use separating axis theorem to test overlap between triangle and box
#  need to test for overlap in these directions:
#  1) the {x,y,z}-directions (actually, since we use the AABB of the triangle
#     we do not even need to test these)
#  2) normal of the triangle
#  3) crossproduct(edge from tri, {x,y,z}-directin)
#     this gives 3x3=9 more tests"""
#def triboxoverlap(float boxcenter[3],float boxhalfsize[3],float triverts[3][3]):
def triboxoverlap(boxcenter, boxhalfsize, triverts):

    # This is the fastest branch on Sun
    # move everything so that the boxcenter is in (0,0,0)
    v0 = minus(triverts[0], boxcenter)
    v1 = minus(triverts[1], boxcenter)
    v2 = minus(triverts[2], boxcenter)

    # compute triangle edges
    e0 = minus(v1, v0) # tri edge 0
    e1 = minus(v2, v1) # tri edge 1
    e2 = minus(v0, v2) # tri edge 2

    # Bullet 3:

    # test the 9 tests first (this was faster)
    fex = abs(e0[0])
    fey = abs(e0[1])
    fez = abs(e0[2])
    if not AXISTEST_X01(e0[2], e0[1], fez, fey, v0, v1, v2, boxhalfsize):
        return False
    if not AXISTEST_Y02(e0[2], e0[0], fez, fex, v0, v1, v2, boxhalfsize):
        return False
    if not AXISTEST_Z12(e0[1], e0[0], fey, fex, v0, v1, v2, boxhalfsize):
        return False

    fex = abs(e1[0])
    fey = abs(e1[1])
    fez = abs(e1[2])
    if not AXISTEST_X01(e1[2], e1[1], fez, fey, v0, v1, v2, boxhalfsize):
        return False
    if not AXISTEST_Y02(e1[2], e1[0], fez, fex, v0, v1, v2, boxhalfsize):
        return False
    if not AXISTEST_Z0(e1[1], e1[0], fey, fex, v0, v1, v2, boxhalfsize):
        return False

    fex = abs(e2[0])
    fey = abs(e2[1])
    fez = abs(e2[2])
    if not AXISTEST_X2(e2[2], e2[1], fez, fey, v0, v1, v2, boxhalfsize):
        return False
    if not AXISTEST_Y1(e2[2], e2[0], fez, fex, v0, v1, v2, boxhalfsize):
        return False
    if not AXISTEST_Z12(e2[1], e2[0], fey, fex, v0, v1, v2, boxhalfsize):
        return False

    # Bullet 1:
    #
    #  first test overlap in the {x,y,z}-directions
    #  find min, max of the triangle each direction, and test for overlap in
    #  that direction -- this is equivalent to testing a minimal AABB around
    #  the triangle against the AABB

    # test in X-direction

    minv, maxv = findminmax(v0[0], v1[0], v2[0])

    if minv > boxhalfsize[0] or maxv < -boxhalfsize[0]:
        return False

    # test in Y-direction

    minv, maxv = findminmax(v0[1], v1[1], v2[1]);

    if minv > boxhalfsize[1] or maxv < -boxhalfsize[1]:
        return False

    # test in Z-direction

    minv, maxv = findminmax(v0[2], v1[2], v2[2]);

    if minv > boxhalfsize[2] or maxv < -boxhalfsize[2]:
        return False

    # Bullet 2:
    #
    #  test if the box intersects the plane of the triangle
    #  compute plane equation of triangle: normal*x+d=0

    normal = cross(e0, e1)

    if not planeboxoverlap(normal, v0, boxhalfsize):
        return False

    return True # box and triangle overlaps

