# recast-navmesh
Navigation-mesh Toolset base on [Recast Navigation](https://github.com/recastnavigation/recastnavigation). Recast Navigation is a excellent toolkit for navigation mesh, but it don't provides any api to load、build、save mesh data(Recast Demo containing all those functionality, just can't use them from command line). And It mix SDL render code with path-finding code etc.

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



## useage

* library

Once build success, the static library `libnavmesh.a` is generated at build directory. There is no install command or share library for now.

```cpp
#include <navmesh.h>

/**
 * load mesh data pre generated from Recast
 * @param path a mesh data file
 */
bool load(const char *path);

/**
 * generated mesh data from a obj/gset file
 * @param from a obj/gset file
 * @param to a mesh data file
 */
bool build(const char *from, const char *to);
```

* tools

```shell
# build mesh data
./tools build test_nav.obj test_nav.mesh

# test path-finding
./tools find test_nav.mesh 1 2 3 9 8 7
```
