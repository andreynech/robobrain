#include <iostream>
#include "geometry3d.h"
#include "montecarlo.h"

/*
location = [1000, 0, 1000, 0] 


event = {
    'triangles': [],
    'particles': [],
    'motion': [0.0, 500.0], # delta_theta, s
    'noise': [30.0, 1.0 * 0.5, 50.0], # bearing, steering, distance
    'measurements': []
}
*/

#define N_PART 2000
#define N_BOX 8
#define N_SENSORS 8

struct measurement_t
{
    btVector3 direction;
    btVector3 inv_direction;
    btVector3 origin;
    btScalar distance;
};

typedef std::vector<measurement_t> measurements_t;

struct request_t
{
    mesh_t mesh;
    particle_vector_t particles;
    motion_t motion;
    motion_noise_t noise;
    measurements_t measurements;
};

std::ostream& operator << (std::ostream &os, const btVector3 &v)
{
    os << "(" << v.x() << "," << v.y() << "," << v.z() << ")";
    return os;
}

std::ostream& operator << (std::ostream &os, const btVector4 &v)
{
    os << "(" << v.x() << "," << v.y() << "," << v.z() << "," << v.w() << ")";
    return os;
}

std::ostream& operator << (std::ostream &os, const box_t &b)
{
    os << "[" << b.first << " " << b.second << "]";
    return os;
}


int main(int argc, char *argv[])
{
    scene = load('obj/room.obj')
    assert len(scene.meshes)
    for mesh in scene.meshes:
        assert len(mesh.vertices)
        for v in mesh.vertices:
            triangles.extend(v)
    release(scene)

    box_t bbox; 
    bounding_box(triangles, bbox);
    btVector3 world_size = bbox.second - bbox.first

    std::cout << "Bounding box:" << bbox << std::end;
    print('World size:', world_x_size, world_y_size, world_z_size)

/*
    particles = []
    if len(particles) == 0:
        for i in range(N_PART):
            #particle = (random.random() * world_x_size + bbox[0][0],
            #            random.random() * 3 + bbox[0][1],
            #            random.random() * world_z_size + bbox[0][2],
            #            random.random() * 2.0 * pi)
            particle = (random.gauss(location[0], 500),
                        random.gauss(location[1], 500),
                        random.gauss(location[2], 500),
                        random.gauss(location[3], pi / 4))

            particles.append(particle)

    vertices = []
    for i in range(0, len(triangles), 9):
        v0 = triangles[i+0:i+3]
        v1 = triangles[i+3:i+6]
        v2 = triangles[i+6:i+9]
        v = (v0, v1, v2,
            g3d.minus(v1, v0), # Find vectors for two edges sharing vert0
            g3d.minus(v2, v0)
        )
        vertices.append(v)

    max_meas = world_x_size **2 + world_z_size ** 2
    weights = [0] * N_PART

    # Split the world evenly with boxes
    box_x_size = world_x_size / N_BOX
    box_y_size = world_y_size
    box_z_size = world_z_size / N_BOX
    box_x_half_size = box_x_size / 2
    box_y_half_size = box_y_size / 2
    box_z_half_size = box_z_size / 2
    boxes = []
    for ix in range(N_BOX):
        for iz in range(N_BOX):
            box = (
                ( # box center
                    box_x_size * ix + box_x_half_size,
                    box_y_half_size,
                    box_z_size * iz + box_z_half_size
                ),
                ( # box half sizes
                    box_x_half_size,
                    box_y_half_size,
                    box_z_half_size
                ),
                [] # placeholder for bounding triangle indexes
            )
            boxes.append(box)

    # Calculate list of bounding triangles for each box
    print('Total triangles:', len(vertices))
    tt = 0
    for box in boxes:
        for i, verts in enumerate(vertices):
            #print(i, verts)
            if g3d.triboxoverlap(box[0], box[1], verts):
                #print(' **** in', i)
                box[2].append(i)
        tt += len(box[2])
        print('Triangles in box:', len(box[2]))
    print('Total triangles in boxes:', tt)

    init_dir = [1.0, 0.0,  0.0]

    for step in range(8):

        # Simulate measurements
        measurements = []
        for s in range(0, N_SENSORS): # generate sensor directions
            measurements.append({'direction': 
                g3d.rotateY(init_dir, 2.0 * pi / N_SENSORS * s)})

        for meas in measurements:
            meas['origin'] = [0.0, 300.0, 0.0]
            min_dist = None
            origin = g3d.plus(location, meas['origin'])
            direction = meas['direction']
            inv_direction = [
                1.0 / (x if abs(x) > 0.00001 else copysign(0.00001, x)) for x in direction
            ]
            meas['inv_direction'] = inv_direction
            #print('origin:', origin, 'dir:', direction)
            triangles_processed = 0
            for bn, box in enumerate(boxes):
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
                    #print('X aabb:', bn, bb, 'cnt:', len(box[2]), 'in direction: ', direction, 'origin', origin)
                    for vx in box[2]:
                        verts = vertices[vx]
                        triangles_processed += 1
                        x, (t,u,v) = g3d.intersect_triangle(origin, 
                                                            direction, 
                                                            verts, 
                                                            True)
                        if x and t > 0:
                            #print(particle, t)
                            if min_dist is None or t < min_dist:
                                min_dist = t
            print('Processed triangles:', triangles_processed)
            meas['distance'] = min_dist
            print('Location/distance:', location, min_dist)

        # Measurement update
        for n, p in enumerate(particles):
            weights[n] = mc.measurement_prob(boxes, 
                                            vertices, 
                                            p, 
                                            measurements, 
                                            noise, 
                                            max_meas)
        #for ww in weights:
        #    print('W:', ww)
        # Normalization
        sum_weights = sum(weights)
        if not sum_weights == 0:
            k = N_PART / sum(weights)
            for n, w in enumerate(weights):
                weights[n] = w * k

        #for i, w in enumerate(weights):
        #    if w > 0:
        #        print('W:', w, particles[i])

        print('Resampling')
        # Resampling
        p2 = []
        index = int(random.random() * N_PART)
        beta = 0.0
        mw = max(weights)
        for i in range(N_PART):
            beta += random.random() * 2.0 * mw
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % N_PART
            p2.append(particles[index])

        est_pos = mc.get_position(particles)
        print('Estimated position:', est_pos)

        visualization(location, est_pos, step, particles, p2, weights)

        particles = p2

        print('======== {0} ========'.format(step))
        #for p in particles:
        #    print('P:', p)

        # Motion update (prediction)
        for n, p in enumerate(particles):
            particles[n] = mc.move(p, motion, noise)

        location[0] += 500
*/

    return 0;
}

