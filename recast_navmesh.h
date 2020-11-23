#pragma once

class dtNavMesh;
class InputGeom;
class rcContext;
class dtQueryFilter;
class dtNavMeshQuery;

/**
 * Recast Navigation mesh toolset for path-finding, building mesh data
 */
class RecastNavMesh
{
public:
    /// These are just sample areas to use consistent values across the samples.
    /// The use should specify these base on his needs.
    enum SamplePolyAreas
    {
        SAMPLE_POLYAREA_GROUND,
        SAMPLE_POLYAREA_WATER,
        SAMPLE_POLYAREA_ROAD,
        SAMPLE_POLYAREA_DOOR,
        SAMPLE_POLYAREA_GRASS,
        SAMPLE_POLYAREA_JUMP,
    };
    enum SamplePolyFlags
    {
        SAMPLE_POLYFLAGS_WALK = 0x01, // Ability to walk (ground, grass, road)
        SAMPLE_POLYFLAGS_SWIM = 0x02, // Ability to swim (water).
        SAMPLE_POLYFLAGS_DOOR = 0x04, // Ability to move through doors.
        SAMPLE_POLYFLAGS_JUMP = 0x08, // Ability to jump.
        SAMPLE_POLYFLAGS_DISABLED = 0x10,  // Disabled polygon
        SAMPLE_POLYFLAGS_ALL      = 0xffff // All abilities.
    };
    enum SamplePartitionType
    {
        SAMPLE_PARTITION_WATERSHED,
        SAMPLE_PARTITION_MONOTONE,
        SAMPLE_PARTITION_LAYERS,
    };

    /**
     * Setting for building mesh data
     * same as RecastDemo Gui left setting area
     */
    struct Setting
    {
        float cellSize;
        float cellHeight;

        float agentMaxSlope;
        float agentHeight;
        float agentMaxClimb;
        float agentRadius;

        float edgeMaxLen;
        float edgeMaxError;
        float regionMinSize;
        float regionMergeSize;
        float vertsPerPoly;

        float detailSampleDist;
        float detailSampleMaxError;

        int partitionType;
    };

public:
    RecastNavMesh(/* args */);
    ~RecastNavMesh();

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
     * ported from RecastDemo void Sample::saveAll(const char* path, const dtNavMesh* mesh)
     */
    bool save(const char *path);

private:
    bool raw_build(InputGeom *geom, rcContext *ctx);

private:
    class dtNavMesh *_nav_mesh;
    class dtNavMeshQuery *_nav_query;

    const float *_poly_pick;
    const struct Setting *_setting;
    const class dtQueryFilter *_filter;
};
