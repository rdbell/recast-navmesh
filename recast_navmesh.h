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

    static const int MAX_POLYS = 256;

public:
    RecastNavMesh();
    explicit RecastNavMesh(const float *poly_pick_ext,
                           const struct Setting *setting,
                           const class dtQueryFilter *filter);
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
     */
    bool save(const char *path);

    /**
     * pathfinding(follow)
     * right-handle coordinate, The upper right corner as the origin, x to right
     * @return the actual point size,0 no path found, <0 error
     */
    int follow(float sx, float sy, float sz, float ex, float ey, float ez,
               float *points, int size, float step = 0.5f);

    /**
     * pathfinding(straight)
     * right-handle coordinate, The upper right corner as the origin, x to right
     * @return the actual point size,0 no path found, <0 error
     */
    int straight(float sx, float sy, float sz, float ex, float ey, float ez,
                 float *points, int size);

private:
    bool raw_build(InputGeom *geom, rcContext *ctx);
    int smooth(float *m_spos, float *m_epos, unsigned int *m_polys,
               int m_npolys, unsigned int m_startRef, float *m_smoothPath,
               int size, float step = 0.5f);

    const float *default_poly_pick_ext() const;
    const Setting *default_setting() const;
    const dtQueryFilter *default_filter() const;

private:
    class dtNavMesh *_nav_mesh;
    class dtNavMeshQuery *_nav_query;

    const float *_poly_pick_ext;
    const struct Setting *_setting;
    const class dtQueryFilter *_filter;
};
