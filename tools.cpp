/**
 * Command line tools for nav mesh
 */

#include <cstring>
#include <iostream>

#include "recast_navmesh.h"

int build(const char *from, const char *to);

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Invalid arguments" << std::endl;
        return -1;
    }

    if (0 == strcmp(argv[1], "build"))
    {
        if (argc < 3)
        {
            std::cerr << "build missing file path" << std::endl;
            return -1;
        }

        return build(argv[2], argc > 3 ? argv[3] : nullptr);
    }
    else if (0 == strcmp(argv[1], "find"))
    {
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
