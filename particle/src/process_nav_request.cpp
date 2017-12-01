#include "process_nav_request.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <random>
#include <future>
#include <atomic>
#define _USE_MATH_DEFINES
#include <math.h>

#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include <ppl.h>

#define N_BOX 8


static const unsigned seed1 = 
    unsigned(std::chrono::system_clock::now().time_since_epoch().count());
static std::default_random_engine generator(seed1);
static std::uniform_real_distribution<btScalar> random_uniform(0.0, 1.0);


template <class B, class F>
void par_for(B begin, B end, F fn) {
  std::atomic<B> idx;
  idx = begin;

  size_t num_cpus = std::thread::hardware_concurrency();
  std::vector<std::future<> > futures(num_cpus);
  for(size_t cpu = 0; cpu != num_cpus; ++cpu) {
    futures[cpu] = std::async(
      std::launch::async,
      [cpu, &idx, end, &fn]() {
        for (;;) {
          B i = idx++;
          if(i >= end) break;
          fn(i, cpu);
        }
      }
    );
  }
  for(size_t cpu = 0; cpu != num_cpus; ++cpu) {
    futures[cpu].get();
  }
}


bool 
processLocRequest(particle::LocRequest &loc_request,
                  particle::EstimatedLocation &response,
                  request_t &request,
                  const void *payload, int payloadlen)
{
    bool ok = loc_request.ParseFromArray(payload, payloadlen);
    if(!ok)
    {
        std::cout << "Deserialization error" << std::endl;
        response.set_error("Deserialization error");
    }
    else
    {
        // Process request

        if(request.mesh.vertices.empty())
        {
            // Create an instance of the Importer class
            Assimp::Importer importer;
            // And have it read the given file with some example postprocessing
            // Usually - if speed is not the most important aspect for you - you'll 
            // propably to request more postprocessing than we do in this example.
            const aiScene* scene = importer.ReadFile(loc_request.map_location(), 
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
                response.set_error(importer.GetErrorString());
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

            bounding_box(request.mesh, request.bbox);
            request.world_size = request.bbox.second - request.bbox.first;
            std::cout << "Bounding box: " << request.bbox << std::endl;
            std::cout << "World size: " << request.world_size << std::endl;
            std::cout << "Vertex count: " << request.mesh.vertices.size() << std::endl;
            std::cout << "Triangle count: " << request.mesh.triangles.size() << std::endl;

            // Split the world evenly with boxes
            btScalar box_x_size = request.world_size.x() / N_BOX;
            btScalar box_y_size = request.world_size.y();
            btScalar box_z_size = request.world_size.z() / N_BOX;
            btScalar box_x_half_size = box_x_size / 2;
            btScalar box_y_half_size = box_y_size / 2;
            btScalar box_z_half_size = box_z_size / 2;
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
                    request.partitions.push_back(part);
                }
            }

            // Calculate list of bounding triangles for each box
            size_t triangles_processed = 0;
            for(space_partition_t &part: request.partitions)
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
                //std::cout << "Triangles in box:" << part.bounding_triangles.size() << std::endl;
            }
            std::cout << "Total triangles in boxes:" << triangles_processed << std::endl;
        }

        std::time_t t = std::time(nullptr);

        std::cout << "Preparing particles " << std::put_time(std::localtime(&t), "%T") << std::endl;
        if(loc_request.particles_size() == 0)
        {
            // Generate uniformly distributed set of particles
            request.particles.resize(N_PART);
            for(auto &particle: request.particles)
            {
                particle.setX(random_uniform(generator) * request.world_size.x() + request.bbox.first.x());
                particle.setY(random_uniform(generator) * 3 + request.bbox.first.y());
                particle.setZ(random_uniform(generator) * request.world_size.z() + request.bbox.first.z());
                particle.setW(random_uniform(generator) * btScalar(M_PI) * 2);

                //std::cout << particle.x() << "\t" << particle.y() << std::endl;
            }
        }
        else
        {
            // Client sent us previous state. Copy it out of gRPC structure.
            request.particles.resize(loc_request.particles_size());
            size_t i = 0;
            for(auto &particle: request.particles)
            {
                const particle::vector4 &new_particle = 
                    loc_request.particles(i);
                particle.setX(new_particle.x());
                particle.setY(new_particle.y());
                particle.setZ(new_particle.z());
                particle.setW(new_particle.w());
                ++i;
            }
        }

        std::vector<btScalar> weights;
        weights.resize(request.particles.size());
        const btVector3 init_dir(1.0, 0.0, 0.0);
        btVector3 origin;
        btVector3 xpoint;

        t = std::time(nullptr);
        std::cout << "Preparing measurements " << std::put_time(std::localtime(&t), "%T") << std::endl;
        // Copy measurements out of gRPC structure
        request.measurements.resize(loc_request.measurements_size());
        int idx = 0;
        for(measurement_t &meas: request.measurements)
        {
            const particle::measurement &new_meas = loc_request.measurements(idx);
            meas.direction.setX(new_meas.direction().x());
            meas.direction.setY(new_meas.direction().y());
            meas.direction.setZ(new_meas.direction().z());

            meas.origin.setX(new_meas.origin().x());
            meas.origin.setY(new_meas.origin().y());
            meas.origin.setZ(new_meas.origin().z());

            meas.distance = new_meas.distance();

            ++idx;
        }

        // Copy motion and noise data out of gRPC structure
        request.motion = {
            loc_request.d_theta(), 
            loc_request.s()}; // delta_theta, s
        request.noise = {
            loc_request.bearing_noise(),
            loc_request.steering_noise(),
            loc_request.distance_noise()}; // bearing, steering, distance

        t = std::time(nullptr);
        std::cout << "Measurement update " << std::put_time(std::localtime(&t), "%T") << std::endl;
        // Measurement update
        //#pragma omp parallel for
        //for(int n = 0; n < request.particles.size(); n++)
        par_for(particle_vector_t::size_type(0), request.particles.size(),
                [&weights, &request](particle_vector_t::size_type idx, size_t cpu) {
                    weights[idx] = measurement_prob(request.partitions, 
                                                    request.mesh, 
                                                    request.particles[idx], 
                                                    request.measurements, 
                                                    request.noise);
                }
                );
        /*
        size_t n = 0;
        for(const particle_t &particle: request.particles)
        {
            weights[n] = measurement_prob(request.partitions, 
                                          request.mesh, 
                                          particle, //request.particles[n], 
                                          request.measurements, 
                                          request.noise);
            //std::cout << weights[n] << std::endl;
            ++n;
        }
        */
        std::cout << "Normalization " << std::put_time(std::localtime(&t), "%T") << std::endl;
        // Normalization
        btScalar sum_weights = btScalar(0.0);
        for(const auto &w: weights)
            sum_weights += w;
        //std::cout << "Sum(weights): " << sum_weights << std::endl;
        if(sum_weights > 0)
        {
            btScalar k = request.particles.size() / sum_weights;
            for(auto &w: weights)
                w *= k;
        }

        t = std::time(nullptr);
        std::cout << "Resampling" << std::put_time(std::localtime(&t), "%T") << std::endl;
        // Resampling
        particle_vector_t p2(request.particles.size());
        particle_vector_t::size_type index = 
            particle_vector_t::size_type(random_uniform(generator)
                                         * request.particles.size());
        btScalar beta = 0.0;
        const btScalar two_mw = 
            btScalar(2.0) * (*std::max_element(weights.begin(), weights.end()));
        for(particle_vector_t::size_type i = 0; i < request.particles.size(); ++i)
        {
            beta += random_uniform(generator) * two_mw;
            while(beta > weights[index])
            {
                beta -= weights[index];
                index = (index + 1) % request.particles.size();
            }
            p2[i] = request.particles[index];
        }

        //for(const auto &p: p2) std::cout << p << std::endl;
        //std::cout << std::endl;

        t = std::time(nullptr);
        std::cout << "Estimating position... " << std::put_time(std::localtime(&t), "%T") << std::endl;
        particle_t est_pos;
        //get_position(request.particles, est_pos);
        get_position(p2, est_pos);
        std::cout << "** Estimated position: " << est_pos << std::endl;

        particle::vector4 *loc = response.mutable_location();
        loc->set_x(est_pos.x());
        loc->set_y(est_pos.y());
        loc->set_z(est_pos.z());
        loc->set_w(est_pos.w());

        //visualization(location, est_pos, step, particles, p2, weights)

        request.particles = p2;

        t = std::time(nullptr);
        std::cout << "Motion update " << std::put_time(std::localtime(&t), "%T") << std::endl;
        // Motion update (prediction)
        for(auto &p: request.particles)
            move(p, request.motion, request.noise, p);

        t = std::time(nullptr);
        std::cout << "Copy particle data back to gRPC " << std::put_time(std::localtime(&t), "%T") << std::endl;
        // Copy particle vector to response to let the client keep
        // current state
        response.clear_particles();
        size_t i = 0;
        for(auto &particle: request.particles)
        {
            particle::vector4 *new_particle = 
                response.add_particles();
            new_particle->set_x(particle.x());
            new_particle->set_y(particle.y());
            new_particle->set_z(particle.z());
            new_particle->set_w(particle.w());
            ++i;
        }

        t = std::time(nullptr);
        std::cout << "Done " << std::put_time(std::localtime(&t), "%T") << std::endl;

        response.set_error(""); // No error
    }

    return ok;
}
