cmake_minimum_required(VERSION 3.0)

project(simulation)

set(CMAKE_CXX_COMPILER "g++")

set (CMKAE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-Os")


find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    simulation_msg
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${OSQP_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

set(PROTO_META_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common/proto)

file(GLOB proto_files ${CMAKE_CURRENT_SOURCE_DIR}/common/proto/*.proto)

find_program(PROTOC protoc)

foreach(FIL ${proto_files})
    get_filename_component(FILE_NAME ${FIL} NAME)
    message("file_name: " ${FILE_NAME})
    set(PROTO_FILE ${FILE_NAME})
    execute_process(
        COMMAND ${PROTOC} -I=${PROTO_META_BASE_DIR} --cpp_out=${PROTO_META_BASE_DIR} ${PROTO_FILE}
    )
endforeach()

file(GLOB_RECURSE cpp_files  
    "common/*.cc"
    "control/*.cc"
    "planning/*.cc"
    "nodes/pc_node/*.cc"
)

file(
    GLOB_RECURSE chassis_files
    "common/*.cc"
    "chassis/*.cc"
    "nodes/chassis_node/*.cc"
)

catkin_package()

add_executable(${PROJECT_NAME}_main_node
 ${cpp_files}
)

add_executable(${PROJECT_NAME}_chassis_node
 ${chassis_files}

)

add_dependencies(${PROJECT_NAME}_main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}_main_node
    ${GLOG_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    glog::glog
    osqp::osqp
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_chassis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}_chassis_node
    ${GLOG_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    glog::glog
    osqp::osqp
    ${catkin_LIBRARIES}
)