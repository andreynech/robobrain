#include "geometry3d.h"
#include <tuple>
#include <algorithm>
#include <cassert>
#include <cmath>

std::ostream& operator << (std::ostream &os, const btVector3 &v)
{
    os << v.x() << " " << v.y() << " " << v.z();
    return os;
}

std::ostream& operator << (std::ostream &os, const btVector4 &v)
{
    os << v.x() << " " << v.y() << " " << v.z() << " " << v.w();
    return os;
}

std::ostream& operator << (std::ostream &os, const box_t &b)
{
    os << "[" << b.first << ", " << b.second << "]";
    return os;
}



// Calculates 3D bounding box of the mesh.
// Sets bbox tuple to (min_coord, max_coord)
void bounding_box(const mesh_t &mesh, box_t &bbox)
{
    assert(!mesh.vertices.empty());

    bbox.first = mesh.vertices[0];
    bbox.second = mesh.vertices[0];

    for(const auto& vert: mesh.vertices)
    {
        // max coordinates
        if(bbox.second.x() < vert.x())
            bbox.second.setX(vert.x());
        if(bbox.second.y() < vert.y())
            bbox.second.setY(vert.y());
        if(bbox.second.z() < vert.z())
            bbox.second.setZ(vert.z());
        // min coordinates
        if(bbox.first.x() > vert.x())
            bbox.first.setX(vert.x());
        if(bbox.first.y() > vert.y())
            bbox.first.setY(vert.y());
        if(bbox.first.z() > vert.z())
            bbox.first.setZ(vert.z());
    }
}


void rotateY(const btVector3 &v, btScalar radians, btVector3 &res)
{
    static const btVector3 axis(0, 1, 0);
    res = v.rotate(axis, radians);
}


// Branchless intersections between axis-aligned bounding (AABB) box and ray.
// 
// For more details:
// https://tavianator.com/fast-branchless-raybounding-box-intersections-part-2-nans/
bool intersection(const box_t &bbox, 
                  const btVector3 &origin, 
                  const btVector3 &dir_inv)
{
    btScalar t1 = (bbox.first.x() - origin.x()) * dir_inv.x();
    btScalar t2 = (bbox.second.x() - origin.x()) * dir_inv.x();
 
    btScalar tmin = std::min(t1, t2);
    btScalar tmax = std::max(t1, t2);
 
    t1 = (bbox.first.y() - origin.y()) * dir_inv.y();
    t2 = (bbox.second.y() - origin.y()) * dir_inv.y();

    tmin = std::max(tmin, std::min(t1, t2));
    tmax = std::min(tmax, std::max(t1, t2));
 
    t1 = (bbox.first.z() - origin.z()) * dir_inv.z();
    t2 = (bbox.second.z() - origin.z()) * dir_inv.z();

    tmin = std::max(tmin, std::min(t1, t2));
    tmax = std::min(tmax, std::max(t1, t2));

    return tmax > std::max(tmin, btScalar(0.0));
}


bool distance(const btVector3 &ro, 
              const btVector3 &rd, 
              const box_t &aabb, 
              btScalar *dist)
{

    btScalar lo = -1e10;
    btScalar hi =  1e10;
    btScalar dimLo, dimHi;

    dimLo = (aabb.first.x() - ro.x()) / rd.x();
    dimHi = (aabb.second.x() - ro.x()) / rd.x();

    if(dimLo > dimHi)
        std::swap(dimLo, dimHi);

    if(dimHi < lo || dimLo > hi)
        return false;

    if(dimLo > lo)
        lo = dimLo;
    if(dimHi < hi)
        hi = dimHi;

    dimLo = (aabb.first.y() - ro.y()) / rd.y();
    dimHi = (aabb.second.y() - ro.y()) / rd.y();

    if(dimLo > dimHi)
        std::swap(dimLo, dimHi);

    if(dimHi < lo || dimLo > hi)
        return false;

    if(dimLo > lo)
        lo = dimLo;
    if(dimHi < hi)
        hi = dimHi;

    dimLo = (aabb.first.z() - ro.z()) / rd.z();
    dimHi = (aabb.second.z() - ro.z()) / rd.z();

    if(dimLo > dimHi)
        std::swap(dimLo, dimHi);

    if(dimHi < lo || dimLo > hi)
        return false;

    if(dimLo > lo)
        lo = dimLo;
    if(dimHi < hi)
        hi = dimHi;

    if(lo > hi)
        return false;

    *dist = lo;
    return true;
}

// https://github.com/stackgl/ray-aabb-intersection
// ro - origin
// rd - direction
bool boxrayintersect(const box_t &aabb,
                     const btVector3 &ro, 
                     const btVector3 &rd,
                     btVector3 &xpoint)
{
    btScalar d;
    if(distance(ro, rd, aabb, &d))
    {
        xpoint.setX(ro.x() + rd.x() * d);
        xpoint.setY(ro.y() + rd.y() * d);
        xpoint.setZ(ro.z() + rd.z() * d);
        return true;
    }

    return false;
}


// Moeller-Trumbore ray-triangle intersection algorithm
// For more details:
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
// and
// http://webserver2.tecgraf.puc-rio.br/~mgattass/cg/trbRR/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
//
// Parameters:
//
// orig - tuple with x,y,z coordinates of the ray origin
//
// dir - unit direction vector of the ray (x,y,z tuple)
//
// vert - triangle to test as tuple (vertex0, vertex1, vertex2, edge1, edge2)
// where each vertex is a tuple of (x,y,z) vertex coordinates and edge1 =
// vert[1] - vert[0], and edge2 = vert[2] - vert[0]
//
// test_cull - if True, triangles are considered single sided. If ray
// intersects triangle from "opposite" side, it is not considered as
// intersection. If False, triangles are considered double-sided.
//
// Returns:
// tuple I, (t, u, v) where I is the Boolean value indicating whether the ray
// intersects the triangle. t is the distance to the intersection. u and v
// are coordinates of the intersection.
bool intersect_triangle(const btVector3 &orig, 
                        const btVector3 &dir,
                        const btVector3 &vert0,
                        const btVector3 &edge1,
                        const btVector3 &edge2,
                        btVector3 &xpoint,
                        bool test_cull)
{
    btScalar t, u, v;
    // To ensure numerical stability the test which eliminates parallel rays
    // must compare the determinant to a small interval around zero.
    btScalar epsilon = btScalar(1e-5);
    // Begin calculating determinant - also used to calculate U parameter
    btVector3 pvec = dir.cross(edge2);
    // If determinant is near zero, ray lies in plane of triangle
    btScalar det = edge1.dot(pvec);

    if(test_cull)
    {
        if(det < epsilon)
            return false;
        // Calculate distance from vert0 to ray origin
        btVector3 tvec = orig - vert0;
        // Calculate U parameter and test bounds
        u = tvec.dot(pvec);
        if(u < 0 || u > det)
            return false;
        // Prepare to test V parameter
        btVector3 qvec = tvec.cross(edge1);
        // Calculate V parameter and test bounds
        v = dir.dot(qvec);
        if(v < 0 || u + v > det)
            return false;
        // Calculate t, scale parameters, ray intersects triangle
        t = edge2.dot(qvec);
        btScalar inv_det = btScalar(1.0) / det;
        t *= inv_det;
        u *= inv_det;
        v *= inv_det;
    }
    else // The non-culling branch
    {
        if(det > -epsilon && det < epsilon)
            return false;
        btScalar inv_det = btScalar(1.0) / det;
        // Calculate distance from vert0 to ray origin
        btVector3 tvec = orig - vert0;
        // Calculate U parameter and test bounds
        u = tvec.dot(pvec) * inv_det;
        if(u < 0 || u > 1.0)
            return false;
        // Prepare to test V parameter
        btVector3 qvec = tvec.cross(edge1);
        // Calculate V parameter and test bounds
        v = dir.dot(qvec) * inv_det;
        if(v < 0 || u + v > 1.0)
            return false;
        // Calculate t, ray intersects triangle
        t = edge2.dot(qvec) * inv_det;
    }

    xpoint.setX(t);
    xpoint.setY(u);
    xpoint.setZ(v);
    return true;
}


// AABB-triangle overlap test code
// by Tomas Akenine-Möller
//
// Function: int triBoxOverlap(float boxcenter[3],
//          float boxhalfsize[3],float triverts[3][3]);
//
// For more details:
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
//

bool planeboxoverlap(const btVector3 &normal,
                     const btVector3 &vert,
                     const btVector3 &maxbox)
{
    btVector3 vmin;
    btVector3 vmax;

    if(normal.x() > 0.0)
    {
        vmin.setX(-maxbox.x() - vert.x());
        vmax.setX( maxbox.x() - vert.x());
    }
    else
    {
        vmin.setX( maxbox.x() - vert.x());
        vmax.setX(-maxbox.x() - vert.x());
    }

    if(normal.y() > 0.0)
    {
        vmin.setY(-maxbox.y() - vert.y());
        vmax.setY( maxbox.y() - vert.y());
    }
    else
    {
        vmin.setY( maxbox.y() - vert.y());
        vmax.setY(-maxbox.y() - vert.y());
    }

    if(normal.z() > 0.0)
    {
        vmin.setZ(-maxbox.z() - vert.z());
        vmax.setZ( maxbox.z() - vert.z());
    }
    else
    {
        vmin.setZ( maxbox.z() - vert.z());
        vmax.setZ(-maxbox.z() - vert.z());
    }

    if(normal.dot(vmin) > 0.0)
        return false;

    if(normal.dot(vmax) >= 0.0)
        return true;

    return false;
}


//======================== X-tests ========================

bool AXISTEST_X01(btScalar a, btScalar b,
                  btScalar fa, btScalar fb, 
                  const btVector3 &v0,
                  const btVector3 &v1,
                  const btVector3 &v2,
                  const btVector3 &boxhalfsize)
{
    btScalar minv, maxv;
    btScalar p0 = a*v0.y() - b*v0.z();
    btScalar p2 = a*v2.y() - b*v2.z();
    if(p0 < p2)
    {
        minv = p0;
        maxv = p2;
    }
    else
    {
        minv = p2;
        maxv = p0;
    }
    btScalar rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
    if(minv > rad || maxv < -rad)
        return false;
    return true;
}

bool AXISTEST_X2(btScalar a, btScalar b,
                 btScalar fa, btScalar fb, 
                 const btVector3 &v0,
                 const btVector3 &v1,
                 const btVector3 &v2,
                 const btVector3 &boxhalfsize)
{
    btScalar minv, maxv;
    btScalar p0 = a*v0.y() - b*v0.z();
    btScalar p1 = a*v1.y() - b*v1.z();
    if(p0 < p1)
    {
        minv = p0;
        maxv = p1;
    }
    else
    {
        minv = p1;
        maxv = p0;
    }
    btScalar rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
    if(minv > rad || maxv < -rad)
        return false;
    return true;
}

//======================== Y-tests ========================

bool AXISTEST_Y02(btScalar a, btScalar b,
                  btScalar fa, btScalar fb, 
                  const btVector3 &v0,
                  const btVector3 &v1,
                  const btVector3 &v2,
                  const btVector3 &boxhalfsize)
{
    btScalar minv, maxv;
    btScalar p0 = -a*v0.x() + b*v0.z();
    btScalar p2 = -a*v2.x() + b*v2.z();
    if(p0 < p2)
    {
        minv = p0;
        maxv = p2;
    }
    else
    {
        minv = p2;
        maxv = p0;
    }
    btScalar rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
    if(minv > rad || maxv < -rad)
        return false;
    return true;
}

bool AXISTEST_Y1(btScalar a, btScalar b,
                 btScalar fa, btScalar fb, 
                 const btVector3 &v0,
                 const btVector3 &v1,
                 const btVector3 &v2,
                 const btVector3 &boxhalfsize)
{
    btScalar minv, maxv;
    btScalar p0 = -a*v0.x() + b*v0.z();
    btScalar p1 = -a*v1.x() + b*v1.z();
    if(p0 < p1)
    {
        minv = p0;
        maxv = p1;
    }
    else
    {
        minv = p1;
        maxv = p0;
    }
    btScalar rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
    if(minv > rad || maxv < -rad)
        return false;
    return true;
}

//======================== Z-tests ========================

bool AXISTEST_Z12(btScalar a, btScalar b,
                  btScalar fa, btScalar fb, 
                  const btVector3 &v0,
                  const btVector3 &v1,
                  const btVector3 &v2,
                  const btVector3 &boxhalfsize)
{
    btScalar minv, maxv;
    btScalar p1 = a*v1.x() - b*v1.y();
    btScalar p2 = a*v2.x() - b*v2.y();
    if(p2 < p1)
    {
        minv = p2;
        maxv = p1;
    }
    else
    {
        minv = p1;
        maxv = p2;
    }
    btScalar rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
    if(minv > rad || maxv < -rad)
        return false;
    return true;
}

bool AXISTEST_Z0(btScalar a, btScalar b,
                 btScalar fa, btScalar fb, 
                 const btVector3 &v0,
                 const btVector3 &v1,
                 const btVector3 &v2,
                 const btVector3 &boxhalfsize)
{
    btScalar minv, maxv;
    btScalar p0 = a*v0.x() - b*v0.y();
    btScalar p1 = a*v1.x() - b*v1.y();
    if(p0 < p1)
    {
        minv = p0;
        maxv = p1;
    }
    else
    {
        minv = p1;
        maxv = p0;
    }
    btScalar rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
    if(minv > rad || maxv < -rad)
        return false;
    return true;
}


template< typename T >
std::tuple<T, T> findminmax( const T& a, const T& b, const T& c )
{
    return std::make_tuple(std::min(std::min(a,b), c), std::max(std::max(a,b), c));
}


// Use separating axis theorem to test overlap between triangle and box
//  need to test for overlap in these directions:
//  1) the {x,y,z}-directions (actually, since we use the AABB of the triangle
//     we do not even need to test these)
//  2) normal of the triangle
//  3) crossproduct(edge from tri, {x,y,z}-directin)
//     this gives 3x3=9 more tests"""
//def triboxoverlap(float boxcenter[3],float boxhalfsize[3],float triverts[3][3]):
bool triboxoverlap(const btVector3 &boxcenter,
                   const btVector3 &boxhalfsize,
                   const trianglevert_t &triverts)
{
    // This is the fastest branch on Sun
    // move everything so that the boxcenter is in (0,0,0)
    btVector3 v0 = *(triverts[0]) - boxcenter;
    btVector3 v1 = *(triverts[1]) - boxcenter;
    btVector3 v2 = *(triverts[2]) - boxcenter;

    // compute triangle edges
    btVector3 e0 = v1 - v0; // tri edge 0
    btVector3 e1 = v2 - v1; // tri edge 1
    btVector3 e2 = v0 - v2; // tri edge 2

    // Bullet 3:

    // test the 9 tests first (this was faster)
    btScalar fex = std::abs(e0.x());
    btScalar fey = std::abs(e0.y());
    btScalar fez = std::abs(e0.z());
    if(!AXISTEST_X01(e0.z(), e0.y(), fez, fey, v0, v1, v2, boxhalfsize))
        return false;
    if(!AXISTEST_Y02(e0.z(), e0.x(), fez, fex, v0, v1, v2, boxhalfsize))
        return false;
    if(!AXISTEST_Z12(e0.y(), e0.x(), fey, fex, v0, v1, v2, boxhalfsize))
        return false;

    fex = std::abs(e1.x());
    fey = std::abs(e1.y());
    fez = std::abs(e1.z());
    if(!AXISTEST_X01(e1.z(), e1.y(), fez, fey, v0, v1, v2, boxhalfsize))
        return false;
    if(!AXISTEST_Y02(e1.z(), e1.x(), fez, fex, v0, v1, v2, boxhalfsize))
        return false;
    if(!AXISTEST_Z0(e1.y(), e1.x(), fey, fex, v0, v1, v2, boxhalfsize))
        return false;

    fex = std::abs(e2.x());
    fey = std::abs(e2.y());
    fez = std::abs(e2.z());
    if(!AXISTEST_X2(e2.z(), e2.y(), fez, fey, v0, v1, v2, boxhalfsize))
        return false;
    if(!AXISTEST_Y1(e2.z(), e2.x(), fez, fex, v0, v1, v2, boxhalfsize))
        return false;
    if(!AXISTEST_Z12(e2.y(), e2.x(), fey, fex, v0, v1, v2, boxhalfsize))
        return false;

    // Bullet 1)
    //
    //  first test overlap in the {x,y,z}-directions
    //  find min, max of the triangle each direction, and test for overlap in
    //  that direction -- this is equivalent to testing a minimal AABB around
    //  the triangle against the AABB

    // test in X-direction

    btScalar minv, maxv;

    std::tie(minv, maxv) = findminmax(v0.x(), v1.x(), v2.x());

    if(minv > boxhalfsize.x() || maxv < -boxhalfsize.x())
        return false;

    // test in Y-direction

    std::tie(minv, maxv) = findminmax(v0.y(), v1.y(), v2.y());

    if(minv > boxhalfsize.y() || maxv < -boxhalfsize.y())
        return false;

    // test in Z-direction

    std::tie(minv, maxv) = findminmax(v0.z(), v1.z(), v2.z());

    if(minv > boxhalfsize.z() || maxv < -boxhalfsize.z())
        return false;

    // Bullet 2)
    //
    //  test if(the box intersects the plane of the triangle
    //  compute plane equation of triangle: normal*x+d=0

    const btVector3 normal = e0.cross(e1);

    if(!planeboxoverlap(normal, v0, boxhalfsize))
        return false;

    return true; // box and triangle overlaps
}

