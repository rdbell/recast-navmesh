cmake_minimum_required(VERSION 3.5)

project(Recast-NavMesh)

# set the original recastnavigation path
set(RECAST_PATH "recastnavigation")

option(RECAST_NAVMESH_TOOLS "Build tools" ON)

################################################################################
file(GLOB SRC_LIST
    "${RECAST_PATH}/Detour/Source/*.cpp"
    "${RECAST_PATH}/Recast/Source/*.cpp"
    "${RECAST_PATH}/Recast/Source/InputGeom.cpp"
    "${RECAST_PATH}/Recast/Source/MeshLoaderObj.cpp"
    "recast_navmesh.cpp"
)

add_library(recast-navmesh STATIC ${SRC_LIST})
target_include_directories(recast-navmesh PRIVATE
    ${RECAST_PATH}/Detour/Include
    ${RECAST_PATH}/Recast/Include
    ${RECAST_PATH}/RecastDemo/Include
)

################################################################################
# link_directories(
#     ${LIBRARY_OUTPUT_PATH}
# )
file(GLOB_RECURSE SRC_LIST "tools.cpp")
add_executable(tools ${SRC_LIST})

target_include_directories(tools PRIVATE

)

target_link_libraries(tools
    recast-navmesh
)

################################################################################
ADD_TEST(exe_test ${PROJECT_BINARY_DIR}/tools)
ENABLE_TESTING()
