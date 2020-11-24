cmake_minimum_required(VERSION 3.5)

project(Recast-NavMesh)

# set the original recastnavigation path
set(RECAST_PATH "${CMAKE_CURRENT_SOURCE_DIR}/recastnavigation")

option(RECAST_NAVMESH_TOOLS "Build tools" ON)

# cmake -DCMAKE_BUILD_TYPE=Strict
set(CMAKE_CXX_STANDARD 11)
if(CMAKE_BUILD_TYPE MATCHES Strict)
    set(CMAKE_CXX_FLAGS "-O0 -g -pipe -Wall -pedantic -std=c++11")
endif()

################################################################################
file(GLOB SRC_LIST
    "${RECAST_PATH}/Detour/Source/*.cpp"
    "${RECAST_PATH}/Recast/Source/*.cpp"
    "${RECAST_PATH}/DebugUtils/Source/DebugDraw.cpp"
    "${RECAST_PATH}/RecastDemo/Source/InputGeom.cpp"
    "${RECAST_PATH}/RecastDemo/Source/MeshLoaderObj.cpp"
    "${RECAST_PATH}/RecastDemo/Source/ChunkyTriMesh.cpp"
    "recast_navmesh.cpp"
)

add_library(recast-navmesh STATIC ${SRC_LIST})
target_include_directories(recast-navmesh PRIVATE
    ${RECAST_PATH}/Detour/Include
    ${RECAST_PATH}/Recast/Include
    ${RECAST_PATH}/DebugUtils/Include
    ${RECAST_PATH}/RecastDemo/Include
)

################################################################################

if (RECAST_NAVMESH_TOOLS)
    file(GLOB_RECURSE SRC_LIST "tools.cpp")
    add_executable(tools ${SRC_LIST})

    target_link_libraries(tools
        recast-navmesh
    )
endif()

################################################################################

# make test CTEST_OUTPUT_ON_FAILURE=TRUE 可输出日志
enable_testing()

add_test(
    NAME build_test
    COMMAND ${PROJECT_BINARY_DIR}/tools
    build
    ${RECAST_PATH}/RecastDemo/Bin/Meshes/nav_test.obj
    ${PROJECT_CURRENT_BINARY_DIR}/nav_test.mesh
)

add_test(
    NAME follow_test
    COMMAND ${PROJECT_BINARY_DIR}/tools
    follow
    ${PROJECT_CURRENT_BINARY_DIR}/nav_test.mesh
    19 -2 -23 -21 -2 29
)

add_test(
    NAME follow_fail_test
    COMMAND ${PROJECT_BINARY_DIR}/tools
    follow
    ${PROJECT_CURRENT_BINARY_DIR}/nav_test.mesh
    -20 4 -13 -21 -2 29
)
set_tests_properties(follow_fail_test PROPERTIES WILL_FAIL TRUE)
