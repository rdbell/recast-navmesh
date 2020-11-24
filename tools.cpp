/**
 * Command line tools for nav mesh
 */

#include <cstring>
#include <iostream>

#include "recast_navmesh.h"

int build(const char *from, const char *to);
int follow(const char *file, float sx, float sy, float sz, float ex, float ey,
           float ez);

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Invalid arguments" << std::endl;
        return -1;
    }

    // tools build nav_test.obj nav_test.mesh
    if (0 == strcmp(argv[1], "build"))
    {
        if (argc < 3)
        {
            std::cerr << "build missing file path" << std::endl;
            return -1;
        }

        return build(argv[2], argc > 3 ? argv[3] : nullptr);
    }
    // tools follow nav_test.mesh 19 -2 -23 -21 -2 29
    else if (0 == strcmp(argv[1], "follow"))
    {
        if (argc < 9)
        {
            std::cerr << "follow missing file path" << std::endl;
            return -1;
        }

        return follow(argv[2], strtof(argv[3], nullptr), strtof(argv[4], nullptr),
                      strtof(argv[5], nullptr), strtof(argv[6], nullptr),
                      strtof(argv[7], nullptr), strtof(argv[8], nullptr));
    }
    else
    {
        std::cerr << "Unknow command" << argv[1] << std::endl;
        return -1;
    }
    return 0;
}

int build(const char *from, const char *to)
{
    RecastNavMesh rnm;

    if (!rnm.build(from))
    {
        std::cerr << "build mesh data from " << from << " fail" << std::endl;
        return -1;
    }

    std::string path(to ? to : from);
    if (!to)
    {
        size_t pos = path.find_last_of(".");
        if (pos != std::string::npos)
        {
            path = path.substr(0, pos);
        }
        path.append(".mesh");
    }

    if (!rnm.save(path.c_str()))
    {
        std::cerr << "save mesh data to " << path << " fail" << std::endl;
        return -1;
    }
    return 0;
}

int follow(const char *file, float sx, float sy, float sz, float ex, float ey,
           float ez)
{
    RecastNavMesh rnm;

    if (!rnm.load(file))
    {
        std::cerr << "load mesh data from " << file << " fail" << std::endl;
        return -1;
    }

    int use_size              = 0;
    static const int max_size = 256;
    float points[max_size]    = {0};

    unsigned int status =
        rnm.follow(sx, sy, sz, ex, ey, ez, points, max_size, use_size, 5.0);
    std::cout << "path follow from (" << sx << "," << sy << "," << sz
              << ") to (" << ex << "," << ey << "," << ez << ")" << std::endl;
    if (!RecastNavMesh::is_succeed(status))
    {
        std::cerr << "    FAIL" << std::endl;
        return -1;
    }

    std::cout.precision(3);
    for (int i = 0; i < use_size; i++)
    {
        float *point = &points[i * 3];
        std::cout << "    " << point[0] << "," << point[1] << "," << point[2]
                  << std::endl;
    }

    return RecastNavMesh::is_partia(status) ? 1 : 0;
}
