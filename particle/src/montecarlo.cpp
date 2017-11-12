#include "montecarlo.h"
#include <chrono>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>


// extract position from a particle set
void get_position(const particle_vector_t &particles, btVector3 &pos)
{
    btScalar x = 0.0;
    btScalar y = 0.0;
    btScalar z = 0.0;
    btScalar orientation = 0;
    btScalar init_orientation = particles.front().w();

    for(const auto &p: particles)
    {
        x += p.x();
        y += p.y();
        z += p.z();
        // orientation is tricky because it is cyclic. By normalizing
        // around the first particle we are somewhat more robust to
        // the 0=2pi problem
        orientation += btScalar((fmod((p.w() - init_orientation + M_PI), (2.0 * M_PI)) 
                        + init_orientation - M_PI));
    }

    particle_vector_t::size_type lenp = particles.size();
    pos.setX(x / lenp);
    pos.setY(y / lenp);
    pos.setZ(z / lenp);
    pos.setW(orientation / lenp);
}


btScalar random_gauss(btScalar mean, btScalar deviation)
{
    static const unsigned seed1 = 
		unsigned(std::chrono::system_clock::now().time_since_epoch().count());
    static std::default_random_engine generator(seed1);
    static std::normal_distribution<btScalar> gauss(0.0f, 1.0f);

    return mean + gauss(generator) * deviation;
}


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
          particle_t &new_p)
{
    btScalar theta = 
        particle.w() + random_gauss(motion.d_theta, noise.steering); 
    btScalar s = random_gauss(motion.s, noise.distance); 

    new_p.setX(particle.x() + s * cos(theta));
    new_p.setY(particle.y());
    new_p.setZ(particle.z() + s * sin(theta));
    new_p.setW(btScalar(fmod(theta, (2.0*M_PI))));
}

/*
def measurement_prob(boxes, vertices, particle, measurements, noise, max_meas)
{
    # calculate the correct measurement
    predicted_measurements = []

    for measurement in measurements:
        min_dist = None
        origin = g3d.plus(particle, measurement['origin'])
        direction = g3d.rotateY(measurement['direction'], particle[3])
        inv_direction = g3d.rotateY(measurement['inv_direction'], particle[3])
        #print('origin:', origin, 'dir:', direction)
        for box in boxes:
            # build "bounding box" style box
            bb = [
                [
                    box[0][0] - box[1][0],
                    box[0][1] - box[1][1],
                    box[0][2] - box[1][2],
                ],
                [
                    box[0][0] + box[1][0],
                    box[0][1] + box[1][1],
                    box[0][2] + box[1][2],
                ]
            ]
            # Check if measurement ray intersects the box.
            # Only if it is we will check relevant triangles.
            if g3d.boxrayintersectBL(bb, origin, inv_direction):
                for vx in box[2]:
                    verts = vertices[vx]
                    x, (t,_,_) = g3d.intersect_triangle(origin, 
                                                        direction, 
                                                        verts, 
                                                        True)
                    if x and t > 0:
                        #print(particle, t)
                        if min_dist is None or t < min_dist:
                            min_dist = t
        if min_dist is None:
            predicted_measurements.append(max_meas)
        else:
            predicted_measurements.append(min_dist)

    # compute errors
    #print('pred:', predicted_measurements)
    error = 1.0
    bearing_noise = noise[0]
    for meas, predicted_meas in zip(measurements, predicted_measurements):
        error_mes = meas['distance'] - predicted_meas
        #print('error_mes:', meas['distance'], predicted_meas, error_mes)
        # update Gaussian
        error *= (exp(- (error_mes ** 2) / (bearing_noise ** 2) / 2.0) / sqrt(2.0 * pi * (bearing_noise ** 2)))
        #print('error:', error)
    return error
}
*/
