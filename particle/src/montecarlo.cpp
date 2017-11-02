#include "geometry3d.h"


// extract position from a particle set
def get_position(p):
    x = 0.0
    y = 0.0
    z = 0.0
    orientation = 0.0
    (_,_,_,init_orientation) = p[0]
    for (px,py,pz,theta) in p:
        x += px
        y += py
        z += pz
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((theta - init_orientation + pi) % (2.0 * pi)) 
                        + init_orientation - pi)
    return (x / len(p), y / len(p), z / len(p), orientation / len(p))


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
def move(particle, motion, noise):
    _, steering_noise, distance_noise = noise
    x, y, z, theta0 = particle
    d_theta, s = motion
    d_theta = random.gauss(d_theta, steering_noise)
    s = random.gauss(s, distance_noise)
    
    theta = theta0 + d_theta;
    x += s * cos(theta)
    z += s * sin(theta)

    return (x, y, z, theta % (2.0*pi))


def measurement_prob(boxes, vertices, particle, measurements, noise, max_meas):
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


def lambda_handler(event, context):
    xpoints = []
    triangles = event['triangles']
    motion = event['motion'] # delta_theta, s
    noise = event['noise'] # bearing, steering, distance
    #logger.info('Processing {0} triangles'.format(len(triangles)/9))
    for measurement in event['measurements']:
        distances = []
        for i in range(0, len(triangles), 9):
            verts = (
                (triangles[i+0],triangles[i+1],triangles[i+2]),
                (triangles[i+3],triangles[i+4],triangles[i+5]),
                (triangles[i+6],triangles[i+7],triangles[i+8]),
            )
            x, (t,u,v) = g3d.intersect_triangle(
                measurement['origin'],
                measurement['direction'], 
                verts, 
                True)
            if x and t > 0:
                distances.append(t)
        if len(distances) > 0:
            # Calculate world coordinates of the intersection
            #logger.info('Origin {0}'.format(measurement['origin']))
            #logger.info('Direction {0}'.format(measurement['direction']))
            #logger.info('Distances {0}'.format(distances))
            xpoint = g3d.plus(measurement['origin'], 
                mul_scalar(measurement['direction'], min(distances)))
            xpoints.append(xpoint)

    return json.dumps(xpoints)

