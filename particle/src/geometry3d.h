#pragma once

#include <LinearMath/btVector3.h>
#include <vector>
#include <array>
#include <utility>


// Vertex list
typedef std::vector<btVector3> vertices_t;
// Fixed size array of indexes for triangle vertices
typedef std::array<vertices_t::size_type, 3> triangleidx_t;
// Three triangle vertex references
typedef std::array<const btVector3*, 3> trianglevert_t;
// List of triangles
typedef std::vector<triangleidx_t> triangles_t;
// Box represented by two corners, where first is "nearest" corner and
// second is "fartherst"
typedef std::pair<btVector3, btVector3> box_t;

struct mesh_t
{
    vertices_t vertices;
    triangles_t triangles;
};


// Calculates 3D bounding box of the mesh.
// Sets bbox tuple to (min_coord, max_coord)
void bounding_box(const mesh_t &mesh, box_t &bbox);

void rotateY(const btVector3 &v, btScalar radians, btVector3 &res);

bool intersection(const box_t &bbox, 
                  const btVector3 &origin, 
                  const btVector3 &dir_inv);

bool distance(const btVector3 &ro, 
              const btVector3 &rd, 
              const box_t &aabb, 
              btScalar *dist);

// https://github.com/stackgl/ray-aabb-intersection
// ro - origin
// rd - direction
bool boxrayintersect(const box_t &aabb,
                     const btVector3 &ro, 
                     const btVector3 &rd,
                     btVector3 &xpoint);

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
                        bool test_cull=true);

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
                     const btVector3 &maxbox);

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
                   const trianglevert_t &triverts);

