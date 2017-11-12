#pragma once

#include <LinearMath/btVector3.h>
#include <array>
#include "geometry3d.h"

// Particle count
#define N_PART  20

typedef btVector4 particle_t;
typedef std::array<particle_t, N_PART> particle_vector_t;
struct motion_t { btScalar d_theta, s; }; 
struct motion_noise_t { btScalar bearing, steering, distance; }; 
struct space_partition_t
{
    box_t box;
    std::vector<triangles_t::size_type> bounding_triangles;
};
typedef std::vector<space_partition_t> partition_vector_t;


// Generates random number with Gaussian distribution
btScalar random_gauss(btScalar mean, btScalar deviation);

// extract position from a particle set
void get_position(const particle_vector_t &particles, btVector3 &pos);

// Here we are using equations for two-wheels differential
// steering system as presented here 
// http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html :
//
// S = (Sr+Sl)/2
// Theta = (Sr-Sl)/2b+theta0
// X = s*cos(theta)+x0
// Y = s*sin(theta)+y0
// Where Sr and Sl is the distance travelled by each wheel and b is the
// distance between wheels (vehicle width)
//
void move(const particle_t &particle,
          const motion_t &motion,
          const motion_noise_t &noise,
          particle_t &new_p);
