#pragma once

#include <particlefilter.grpc.pb.h>
#include "montecarlo.h"


struct request_t
{
    mesh_t mesh;
    particle_vector_t particles;
    motion_t motion;
    motion_noise_t noise;
    measurements_t measurements;
    box_t bbox; 
    btVector3 world_size;
    partition_vector_t partitions;
};


bool
processLocRequest(particle::LocRequest &loc_request,
                  particle::EstimatedLocation &response,
                  request_t &request,
                  const void *payload, int payloadlen);
