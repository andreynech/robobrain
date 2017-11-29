#include <iostream>
#include <cassert>
#include <chrono>
#include <random>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <limits>

#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include "geometry3d.h"
#include "montecarlo.h"
#include "rpc_over_mqtt.h"

#include <particlefilter.grpc.pb.h>

#define N_BOX 8
#define N_SENSORS 4


struct request_t
{
    mesh_t mesh;
    particle_vector_t particles;
    motion_t motion;
    motion_noise_t noise;
    measurements_t measurements;
};

static const btScalar M_2PI = btScalar(M_PI) * 2;



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


class ParticleServer : public RPCServer
{
public:
    ParticleServer(const std::string &svr_id)
        : RPCServer(svr_id)
    {
        //request.motion = {0.0, 500.0}; // delta_theta, s
        //request.noise = {30.0, 1.0 * 0.5, 50.0}; // bearing, steering, distance
    }


    virtual ~ParticleServer() {}

public:

    virtual void on_message(const struct mosquitto_message *message)
    {
        particle::LocRequest loc_request;
        particle::EstimatedLocation response;

        bool ok = loc_request.ParseFromArray(message->payload, message->payloadlen);
        if(!ok)
        {
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

                bounding_box(request.mesh, bbox);
                world_size = bbox.second - bbox.first;
                std::cout << "Bounding box: " << bbox << std::endl;
                std::cout << "World size: " << world_size << std::endl;
                std::cout << "Vertex count: " << request.mesh.vertices.size() << std::endl;
                std::cout << "Triangle count: " << request.mesh.triangles.size() << std::endl;

                // Split the world evenly with boxes
                btScalar box_x_size = world_size.x() / N_BOX;
                btScalar box_y_size = world_size.y();
                btScalar box_z_size = world_size.z() / N_BOX;
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
                    //std::cout << "Triangles in box:" << part.bounding_triangles.size() << std::endl;
                }
                std::cout << "Total triangles in boxes:" << triangles_processed << std::endl;
            }

            static const unsigned seed1 = 
                unsigned(std::chrono::system_clock::now().time_since_epoch().count());
            std::default_random_engine generator(seed1);
            std::uniform_real_distribution<btScalar> random_uniform(0.0, 1.0);

            btVector4 location(1000, 0, 1000, 0); 

            if(request.particles.empty())
            {
                request.particles.resize(N_PART);
                for(auto &particle: request.particles)
                {
                    particle.setX(random_uniform(generator) * world_size.x() + bbox.first.x());
                    particle.setY(random_uniform(generator) * 3 + bbox.first.y());
                    particle.setZ(random_uniform(generator) * world_size.z() + bbox.first.z());
                    particle.setW(random_uniform(generator) * M_2PI);

                    //particle.setX(random_gauss(location.x(), 500));
                    //particle.setY(random_gauss(location.y(), 500));
                    //particle.setZ(random_gauss(location.z(), 500));
                    //particle.setW(random_gauss(location.w(), M_PI_4));
                    

                    //std::cout << particle.x() << "\t" << particle.y() << std::endl;
                }
            }


            std::array<btScalar, N_PART> weights;
            const btVector3 init_dir(1.0, 0.0, 0.0);
            //btScalar min_dist;
            btVector3 origin;
            btVector3 xpoint;

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

            // Measurement update
            size_t n = 0;
            for(const particle_t &particle: request.particles)
            {
                weights[n] = measurement_prob(partitions, 
                                              request.mesh, 
                                              particle, 
                                              request.measurements, 
                                              request.noise);
                //std::cout << weights[n] << std::endl;
                ++n;
            }

            // Normalization
            btScalar sum_weights = btScalar(0.0);
            for(const auto &w: weights)
                sum_weights += w;
            std::cout << "Sum(weights): " << sum_weights << std::endl;
            if(sum_weights > 0)
            {
                btScalar k = request.particles.size() / sum_weights;
                for(auto &w: weights)
                    w *= k;
            }

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

            //std::cout << "======== " << step << " ========" << std::endl;

            // Motion update (prediction)
            for(auto &p: request.particles)
                move(p, request.motion, request.noise, p);

            location.setX(location.x() + 500);

            response.set_error(""); // No error
        }

        std::string buffer;
        ok = response.SerializeToString(&buffer);
        if(!ok)
        {
            std::cout << "Response serialization error" << std::endl;
            return;
        }

        // Search for client_id string in topic which is in format
        // /server_id/client_id/request
        // Answer should be sent to the topic
        // /server_id/client_id/response
        const std::string topic(message->topic);
        size_t first_slash = topic.rfind("/");
        size_t second_slash = topic.rfind("/", first_slash - 1);
        const std::string client_id = topic.substr(second_slash, first_slash - second_slash);
        const std::string publish_topic = 
            std::string("/") + server_id + client_id 
            + std::string("/response");
        std::cout << "Publish topic: " << publish_topic << std::endl;

        int mid = 0;
        int res = publish(&mid, 
                          publish_topic.c_str(),
                          buffer.size(),
                          buffer.c_str());
        if(res)
            std::cerr << "Publish returned: " << res << " " << mosqpp::strerror(res) << std::endl;
        else
            std::cerr << "Publish message id: " << mid << std::endl;
    }

protected:
	request_t request;
    box_t bbox; 
    btVector3 world_size;
    partition_vector_t partitions;
};


int main(int argc, char *argv[])
{
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

	mosquitto_lib_init();

	ParticleServer server("server1");
    server.run();

	return 0;
}

