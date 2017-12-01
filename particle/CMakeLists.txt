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

if (MSVC)
  add_definitions(-D_WIN32_WINNT=0x600)
endif()

set(BULLET_LIBRARIES LinearMath)
set(ASSIMP_LIBRARIES assimp)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/protos-src)

include_directories(${grpc_SOURCE_DIR}/third_party/googletest/googletest/include
                    ${grpc_SOURCE_DIR}/third_party/googletest/googlemock/include
                    ${grpc_SOURCE_DIR}/third_party/protobuf/src
                    ${grpc_SOURCE_DIR}/include
                    ${assimp_SOURCE_DIR}/include
                    ${assimp_BINARY_DIR}/include
                    ${bullet_SOURCE_DIR}/src
                    ${PROJECT_BINARY_DIR}/protos-src)

link_directories(${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
                 ${grpc_BINARY_DIR}/googletest
                 ${grpc_BINARY_DIR}/googlemock
                 ${bullet_SOURCE_DIR}/src/LinearMath)

set(CLI_SOURCES
    src/rpc_over_mqtt.h
    src/geometry3d.cpp src/geometry3d.h
    src/montecarlo.cpp src/montecarlo.h
    src/test_particle.cpp)

set(PB_SRCS
    protos-src/particlefilter.pb.cc
    protos-src/particlefilter.pb.h)

set(GRPC_SRCS
    protos-src/particlefilter.grpc.pb.cc
    protos-src/particlefilter.grpc.pb.h)

set(PROTO_FILE particlefilter.proto)

add_custom_command(
    OUTPUT ${PB_SRCS}
    COMMAND "${grpc_BINARY_DIR}/third_party/protobuf/$<CONFIG>/protoc"
    ARGS --cpp_out=${PROJECT_BINARY_DIR}/protos-src --proto_path=${PROJECT_SOURCE_DIR}/protos ${PROJECT_SOURCE_DIR}/protos/${PROTO_FILE}
    DEPENDS ${PROJECT_SOURCE_DIR}/protos/${PROTO_FILE}
    COMMENT "Running C++ protocol buffer compiler on ${PROTO_FILE}"
    VERBATIM)

add_custom_command(
    OUTPUT ${GRPC_SRCS}
    COMMAND "${grpc_BINARY_DIR}/third_party/protobuf/$<CONFIG>/protoc"
    ARGS --grpc_out=${PROJECT_BINARY_DIR}/protos-src --proto_path=${PROJECT_SOURCE_DIR}/protos --plugin=protoc-gen-grpc=${grpc_BINARY_DIR}/Debug/grpc_cpp_plugin${CMAKE_EXECUTABLE_SUFFIX} ${PROJECT_SOURCE_DIR}/protos/${PROTO_FILE}
    DEPENDS ${PROJECT_SOURCE_DIR}/protos/${PROTO_FILE}
    COMMENT "Running C++ gRPC compiler on ${PROTO_FILE}"
    VERBATIM)

set(SOURCES
    src/rpc_over_mqtt.h
    src/geometry3d.cpp src/geometry3d.h
    src/montecarlo.cpp src/montecarlo.h
    src/process_nav_request.cpp src/process_nav_request.h
    src/main.cpp)

add_executable(particlefilter ${SOURCES} ${PB_SRCS} ${GRPC_SRCS})
target_link_libraries(particlefilter 
                      ${BULLET_LIBRARIES}
                      ${ASSIMP_LIBRARIES}

                      debug libprotobuf${protobuf_DEBUG_POSTFIX}
                      debug grpc${CMAKE_DEBUG_POSTFIX}
                      debug grpc++${CMAKE_DEBUG_POSTFIX}

                      optimized libprotobuf
                      optimized grpc
                      optimized grpc++

                      mosquitto
                      mosquittopp
                     )

add_executable(test_particle ${CLI_SOURCES} ${PB_SRCS} ${GRPC_SRCS})

target_link_libraries(test_particle 
                      ${BULLET_LIBRARIES}
                      ${ASSIMP_LIBRARIES}

                      debug libprotobuf${protobuf_DEBUG_POSTFIX}
                      debug grpc${CMAKE_DEBUG_POSTFIX}
                      debug grpc++${CMAKE_DEBUG_POSTFIX}

                      optimized libprotobuf
                      optimized grpc
                      optimized grpc++

                      mosquitto
                      mosquittopp
                     )

enable_testing()

# Test geometry3d
add_executable(test_geometry
               src/geometry3d.cpp src/geometry3d.h
               src/test_geometry.cpp)
target_link_libraries(test_geometry
                debug gtest${CMAKE_DEBUG_POSTFIX}
                debug gmock_main${CMAKE_DEBUG_POSTFIX}
                optimized gtest
                optimized gmock_main)
add_test(NAME test_geometry COMMAND test_geometry)

