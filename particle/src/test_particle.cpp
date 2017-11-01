#!/usr/bin/env python3

import geometry3d as g3d
import montecarlo as mc
from pyassimp import *
from math import *
import random
import matplotlib.pyplot as plt


def visualization(robot, est_pos, step, particles, particles_resampled, weights):
    """ Visualization
    :param robot:   the current robot object
    :param step:    the current step
    :param p:       list with particles
    :param pr:      list of resampled particles
    :param weights: particle weights
    """
 
    plt.figure("Robot in the world", figsize=(10., 10.))
    plt.title('Particle filter, step ' + str(step))
 
    # draw coordinate grid for plotting
    grid = [bbox[0][0], bbox[1][0], bbox[0][2], bbox[1][2]]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(int(bbox[0][0]), int(bbox[1][0]), 500)])
    plt.yticks([i for i in range(int(bbox[0][2]), int(bbox[1][2]), 500)])
 
    # draw particles
    for p in particles:
        #print('P:', p) 
        # particle
        circle = plt.Circle((p[0], p[2]), 20., facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)
 
        # particle's orientation
        arrow = plt.Arrow(p[0], p[2], 50*cos(p[3]), 50*sin(p[3]), alpha=1., facecolor='#994c00', edgecolor='#994c00')
        plt.gca().add_patch(arrow)
 
    # draw resampled particles
    for pr in particles_resampled:
 
        # particle
        circle = plt.Circle((pr[0], pr[2]), 20., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)
 
        # particle's orientation
        arrow = plt.Arrow(pr[0], pr[2], 50*cos(pr[3]), 50*sin(pr[3]), alpha=1., facecolor='#006600', edgecolor='#006600')
        plt.gca().add_patch(arrow)
 
    # fixed landmarks of known locations
    #for lm in landmarks:
    #    circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
    #    plt.gca().add_patch(circle)
 
    # robot's location
    circle = plt.Circle((robot[0], robot[2]), 20., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)
 
    # robot's orientation
    arrow = plt.Arrow(robot[0], robot[2], 50*cos(robot[3]), 50*sin(robot[3]), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)
 
    # estimated robot's location
    circle = plt.Circle((est_pos[0], est_pos[2]), 20., facecolor='#ff0000', edgecolor='#0000cc')
    plt.gca().add_patch(circle)
 
    # estimated robot's orientation
    arrow = plt.Arrow(est_pos[0], est_pos[2], 50*cos(est_pos[3]), 50*sin(est_pos[3]), alpha=0.5, facecolor='#ff0000', edgecolor='#000000')
    plt.gca().add_patch(arrow)

    #plt.show()
    plt.savefig("figure_" + str(step) + ".png")
    plt.close()


location = [1000, 0, 1000, 0] 

#{ x: 1000, y: 300, z: -4200 }               1200
#{ x: 1500, y: 300, z: -6906.748046875 }     3906.748046875
#{ x: 2000, y: 300, z: -3903.758083014863 }  903.7580830148631
#{ x: 2500, y: 300, z: -3802.8835927798614 } 802.8835927798614
#{ x: 3000, y: 300, z: -4714.183137763199 }  1714.1831377631988
#{ x: 3500, y: 300, z: -4294.633381125961 }  1294.633381125961
#{ x: 4000, y: 300, z: -4851.1315041878415 } 1851.1315041878415
#{ x: 4500, y: 300, z: -6906.748046875 }     3906.748046875

event = {
    'triangles': [],
    'particles': [],
    'motion': [0.0, 500.0], # delta_theta, s
    'noise': [30.0, 1.0 * 0.5, 50.0], # bearing, steering, distance
    'measurements': []
}


triangles = event['triangles']
particles = event['particles']
motion = event['motion']
noise = event['noise']
measurements = event['measurements']


scene = load('obj/room.obj')
assert len(scene.meshes)
for mesh in scene.meshes:
    assert len(mesh.vertices)
    for v in mesh.vertices:
        triangles.extend(v)
release(scene)

bbox = g3d.bounding_box(triangles)
world_x_size = bbox[1][0] - bbox[0][0]
world_y_size = bbox[1][1] - bbox[0][1]
world_z_size = bbox[1][2] - bbox[0][2]

print('Bounding box:', bbox[0], bbox[1])
print('World size:', world_x_size, world_y_size, world_z_size)

N_PART = 2000
N_BOX = 8
N_SENSORS = 8

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
