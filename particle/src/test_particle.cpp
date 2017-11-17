#include <iostream>
#include <cassert>
#include <chrono>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <limits>

#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include "geometry3d.h"
#include "montecarlo.h"

static const btScalar M_2PI = btScalar(M_PI) * 2;

static const btVector4 location(1000, 0, 1000, 0); 

/*
event = {
    'triangles': [],
    'particles': [],
    'motion': [0.0, 500.0], # delta_theta, s
    'noise': [30.0, 1.0 * 0.5, 50.0], # bearing, steering, distance
    'measurements': []
}
*/

#define N_BOX 2
#define N_SENSORS 8

struct measurement_t
{
    btVector3 direction;
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
    os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return os;
}

std::ostream& operator << (std::ostream &os, const btVector4 &v)
{
    os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << ")";
    return os;
}

std::ostream& operator << (std::ostream &os, const box_t &b)
{
    os << "[" << b.first << " " << b.second << "]";
    return os;
}


int main(int argc, char *argv[])
{
	request_t request;

    {
    // Create an instance of the Importer class
    Assimp::Importer importer;
    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll 
    // propably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile("room.obj", 
		           //aiProcess_FlipWindingOrder       |
                   //aiProcess_CalcTangentSpace       | 
                   aiProcess_Triangulate            |
                   aiProcess_JoinIdenticalVertices  |
                   
		           //aiProcess_PreTransformVertices   |
                   aiProcess_OptimizeGraph          |
                   aiProcess_OptimizeMeshes         |

                   aiProcess_FixInfacingNormals     |
                   aiProcess_GenSmoothNormals       |

                   aiProcess_SortByPType);

    // If the import failed, report it
    if(!scene)
    {
        std::cout << importer.GetErrorString() << std::endl;
        return 1;
    }

    //std::cout << scene->mNumMeshes << " " << scene->mRootNode->mNumMeshes << std::endl;

    unsigned int idx_offset = 0;

	for(unsigned int i = 0; i < scene->mNumMeshes; ++i)
	{
        const aiMesh &mesh = *(scene->mMeshes[i]);
        //std::cout << mesh.mNumVertices << std::endl;
        for(unsigned int v = 0; v < mesh.mNumVertices; ++v)
        {
            btVector3 vert(mesh.mVertices[v].x,
                           mesh.mVertices[v].y,
                           mesh.mVertices[v].z);
            request.mesh.vertices.push_back(vert);
        }
        for(unsigned int f = 0; f < mesh.mNumFaces; ++f)
        {
            const aiFace &face = mesh.mFaces[f];
            assert(face.mNumIndices == 3);
            triangleidx_t new_triangle = {face.mIndices[0] + idx_offset,
                                          face.mIndices[1] + idx_offset,
                                          face.mIndices[2] + idx_offset};
            request.mesh.triangles.push_back(new_triangle);

            const btVector3 &v0 = request.mesh.vertices[new_triangle[0]];
            const btVector3 &v1 = request.mesh.vertices[new_triangle[1]];
            const btVector3 &v2 = request.mesh.vertices[new_triangle[2]];
            request.mesh.edges.push_back({v1-v0, v2-v0});
        }
        idx_offset += mesh.mNumVertices;
	}
    }

    box_t bbox; 
    bounding_box(request.mesh, bbox);
	btVector3 world_size = bbox.second - bbox.first;

    std::cout << "Bounding box: " << bbox << std::endl;
	std::cout << "World size: " << world_size << std::endl;
	std::cout << "Vertex count: " << request.mesh.vertices.size() << std::endl;
	std::cout << "Triangle count: " << request.mesh.triangles.size() << std::endl;

    static const unsigned seed1 = 
		unsigned(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator(seed1);
    std::uniform_real_distribution<btScalar> random_uniform(0.0, 1.0);

    particle_vector_t particles;
    for(auto &particle: particles)
    {
        particle.setX(random_uniform(generator) * world_size.x() + bbox.first.x());
        particle.setY(random_uniform(generator) * 3 + bbox.first.y());
        particle.setZ(random_uniform(generator) * world_size.z() + bbox.first.z());
        particle.setW(random_uniform(generator) * M_2PI);
        /*
        particle.setX(random_gauss(location.x(), 500));
        particle.setY(random_gauss(location.y(), 500));
        particle.setZ(random_gauss(location.z(), 500));
        particle.setW(random_gauss(location.w(), M_PI_4));
        */
        //std::cout << particle << std::endl;
    }

    btScalar max_meas = world_size.x() * world_size.x()
                        + world_size.z() * world_size.z();

    // Split the world evenly with boxes
    btScalar box_x_size = world_size.x() / N_BOX;
    btScalar box_y_size = world_size.y();
    btScalar box_z_size = world_size.z() / N_BOX;
    btScalar box_x_half_size = box_x_size / 2;
    btScalar box_y_half_size = box_y_size / 2;
    btScalar box_z_half_size = box_z_size / 2;
    partition_vector_t partitions;
    for(size_t ix = 0; ix < N_BOX; ++ix)
    {
        for(size_t iz = 0; iz < N_BOX; ++iz)
        {
            space_partition_t part =
            {
                std::make_pair(
                    btVector3( // box center
                        box_x_size * ix + box_x_half_size,
                        box_y_half_size,
                        box_z_size * iz + box_z_half_size
                    ),
                    btVector3( // box half sizes
                        box_x_half_size,
                        box_y_half_size,
                        box_z_half_size
                    )),
                    {} // placeholder for bounding triangle indexes
            };
            partitions.push_back(part);
        }
    }

    // Calculate list of bounding triangles for each box
    size_t triangles_processed = 0;
    for(space_partition_t &part: partitions)
    {
        size_t i = 0;
        for(auto const &tri_indexes: request.mesh.triangles)
        {
            trianglevert_t verts = {
                &request.mesh.vertices[tri_indexes[0]],
                &request.mesh.vertices[tri_indexes[1]],
                &request.mesh.vertices[tri_indexes[2]]
            };
            if(triboxoverlap(part.box.first, part.box.second, verts))
            {
                part.bounding_triangles.push_back(i);
            }
            ++i;
        }
        triangles_processed += part.bounding_triangles.size();
        std::cout << "Triangles in box:" 
                  << part.bounding_triangles.size() << std::endl;
    }
    std::cout << "Total triangles in boxes:" << triangles_processed << std::endl;

    std::array<btScalar, N_PART> weights;
    const btVector3 init_dir(1.0, 0.0, 0.0);
    request.measurements.resize(N_SENSORS);
    btScalar min_dist;
    btVector3 origin;
    btVector3 xpoint;

    for(size_t step = 0; step < 1; ++step)
    {
        // Simulate measurements
        for(size_t s = 0; s < N_SENSORS; ++s) // generate sensor directions
        {
            rotateY(init_dir, M_2PI / N_SENSORS * s, request.measurements[s].direction);
        }

        size_t m = 0;
        for(measurement_t &meas: request.measurements)
        {
            ++m;
            std::cout << "*************** Step: " << step << "  Measurement: " << m << std::endl;
            meas.origin = btVector3(0.0, 300.0, 0.0);
            origin = location + meas.origin;
            std::cout << "Origin: " << origin 
                      << "  direction: " <<  meas.direction 
                      << std::endl;

            min_dist = (-std::numeric_limits<btScalar>::infinity());
            triangles_processed = 0;
            for(const auto &part: partitions)
            {
				std::cout << "Triangles in partition: " << part.bounding_triangles.size() << std::endl;
                // build "bounding box" style box
                // .first is the middle of the box
                // .second is half size
                box_t bb = std::make_pair(
                        part.box.first - part.box.second,
                        part.box.first + part.box.second);

				/*std::cout << "Checking bounding box: " << bb
					<< " (" << part.bounding_triangles.size() << " triangles)" 
					<< std::endl;*/

                // Check if measurement ray intersects the box.
                // Only if it does, we will check relevant triangles.
                if(boxrayintersect(bb, origin, meas.direction, xpoint))
                {
                    //std:: cout << "       - intersection" << std::endl;

					for(const auto &vx: part.bounding_triangles)
                    {
                        const triangleidx_t &tri_verts_idx = 
                            request.mesh.triangles[vx];
                        const btVector3 &vert0 = 
                            request.mesh.vertices[tri_verts_idx[0]];

                        const btVector3 &edge1 = request.mesh.edges[vx][0];
                        const btVector3 &edge2 = request.mesh.edges[vx][1];

                        triangles_processed += 1;
                        bool x = intersect_triangle(origin, meas.direction, 
                                                    vert0, edge1, edge2,
                                                    xpoint, false);
                        const btVector3 &vert1 = 
                            request.mesh.vertices[tri_verts_idx[1]];
                        const btVector3 &vert2 = 
                            request.mesh.vertices[tri_verts_idx[2]];
                        
                        btScalar t = xpoint.x();
                        if(x && t > 0)
                        {
							std::cout << "Intersection with tirangle # " << vx << std::endl;
                            if(min_dist == (-std::numeric_limits<btScalar>::infinity()) 
                               || t < min_dist)
                            {
                                min_dist = t;
                            }
                        }
                    }
                }
            }
            meas.distance = min_dist;
            std::cout << "Processed triangles: " << triangles_processed << std::endl;
            std::cout << "Location/distance: " << location << " / " << min_dist << std::endl;
        }
/*
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
    }

    return 0;
}

