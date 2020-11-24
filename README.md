# recast-navmesh
Navigation-mesh Toolset base on [Recast Navigation](https://github.com/recastnavigation/recastnavigation). Recast Navigation is a excellent toolkit for navigation mesh, but it don't provides any simple api to load、build、save mesh data(Recast Demo containing all those functionality, just can't use them as api or call from command line).

So here this tool comes.

## build
```shell
git clone https://github.com/changnet/recast-navmesh.git
cd recast-navmesh
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
make test
```

## usage

* library

Once build success, the static library `libnavmesh.a` is generated at build directory. There is no install command or share library for now, static link should be enough.

```cpp
#include <recast_navmesh.h>

class RecastNavMesh
{
    /**
     * load mesh data pre generated from Recast
     * @param path a mesh data file
     */
    bool load(const char *path);

    /**
     * generated mesh data from a obj/gset file
     * @param from a obj/gset file
     */
    bool build(const char *from);

    /**
     * save mesh data to file
     */
    bool save(const char *path);

    /**
     * pathfinding(follow)
     * right-handle coordinate, x axis right, y axis up
     * @return status, use is_xx function to check fail.
     */
    unsigned int follow(float sx, float sy, float sz, float ex, float ey,
                        float ez, float *points, int max_size, int &use_size,
                        float step = 0.5f);

    /**
     * pathfinding(straight)
     * right-handle coordinate, x axis right, y axis up
     * @param option Query options. (see: #dtStraightPathOptions)
     * @return status, use is_xx function to check fail.
     */
    unsigned int straight(float sx, float sy, float sz, float ex, float ey,
                          float ez, float *points, int max_size, int &use_size,
                          int option = 0);
};
```
with those api, It's much easier to to load mesh data or path-finding, more detail at example [tools.cpp](tools.cpp).

* tools

```shell
# build mesh data
./tools build test_nav.obj test_nav.mesh

# test path-finding
./tools follow test_nav.mesh 1 2 3 9 8 7
```

Tools allow batch building mesh data from obj file, and do some base test.
