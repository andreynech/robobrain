cmake_minimum_required(VERSION 3.2)

project(particlefilter)

include(CTest)

set(CMAKE_CXX_STANDARD 11)

set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")

include(DownloadProject.cmake)

# Download and configure Bullet
# Do not build Bullet's gmock/gtest and examples/demos
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
download_project(PROJ                bullet
                 GIT_REPOSITORY      https://github.com/bulletphysics/bullet3
                 GIT_TAG             2.87
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
add_subdirectory(${bullet_SOURCE_DIR} ${bullet_BINARY_DIR})


# Download and configure googletest
#download_project(PROJ                googletest
#                 GIT_REPOSITORY      https://github.com/google/googletest.git
#                 GIT_TAG             master
#                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
#)
# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
download_project(PROJ                assimp
                 GIT_REPOSITORY      https://github.com/assimp/assimp.git
                 GIT_TAG             v4.0.1
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
add_subdirectory(${assimp_SOURCE_DIR} ${assimp_BINARY_DIR})

# GRPC
download_project(PROJ                grpc
                 GIT_REPOSITORY      https://github.com/grpc/grpc.git
                 GIT_TAG             v1.7.x
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
add_subdirectory(${grpc_SOURCE_DIR} ${grpc_BINARY_DIR})


set(BULLET_LIBRARIES LinearMath)
set(ASSIMP_LIBRARIES assimp)

include_directories(${grpc_SOURCE_DIR}/third_party/googletest/googletest/include
                    ${grpc_SOURCE_DIR}/third_party/googletest/googlemock/include
                    ${assimp_SOURCE_DIR}/include
                    ${assimp_BINARY_DIR}/include
                    ${bullet_SOURCE_DIR}/src)

link_directories(${grpc_BINARY_DIR}/googletest
                 ${grpc_BINARY_DIR}/googlemock
                 ${bullet_SOURCE_DIR}/src/LinearMath)

set(SOURCES
    src/rpc_over_mqtt.h
    src/geometry3d.cpp src/geometry3d.h
    src/montecarlo.cpp src/montecarlo.h
    src/main.cpp)

add_executable(particlefilter ${SOURCES})
target_link_libraries(particlefilter 
                      ${BULLET_LIBRARIES}
                      ${ASSIMP_LIBRARIES}
                      mosquitto
                      mosquittopp
                     )

set(CLI_SOURCES
    src/rpc_over_mqtt.h
    src/geometry3d.cpp src/geometry3d.h
    src/montecarlo.cpp src/montecarlo.h
    src/test_particle.cpp)

add_executable(test_particle ${CLI_SOURCES})
target_link_libraries(test_particle 
                      ${BULLET_LIBRARIES}
                      ${ASSIMP_LIBRARIES}
                      mosquitto
                      mosquittopp
                     )

#enable_testing()

# Test geometry3d
add_executable(test_geometry
               src/geometry3d.cpp src/geometry3d.h
               src/test_geometry.cpp)
target_link_libraries(test_geometry gtest gmock_main)
add_test(NAME test_geometry COMMAND test_geometry)

