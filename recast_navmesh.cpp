#include <Recast.h>
#include <iostream>
#include <InputGeom.h>
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourNavMeshBuilder.h>

#include <cmath>
#include <cstring> /* for memset */

#include "recast_navmesh.h"

////////////////////////////////////////////////////////////////////////////////
// Those code are ported from Recast Navigation, please Keep them consistent

static const int NAVMESHSET_MAGIC =
    'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader
{
    dtTileRef tileRef;
    int dataSize;
};

inline bool inRange(const float *v1, const float *v2, const float r, const float h)
{
    const float dx = v2[0] - v1[0];
    const float dy = v2[1] - v1[1];
    const float dz = v2[2] - v1[2];
    return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
}

static int fixupCorridor(dtPolyRef *path, const int npath, const int maxPath,
                         const dtPolyRef *visited, const int nvisited)
{
    int furthestPath    = -1;
    int furthestVisited = -1;

    // Find furthest common polygon.
    for (int i = npath - 1; i >= 0; --i)
    {
        bool found = false;
        for (int j = nvisited - 1; j >= 0; --j)
        {
            if (path[i] == visited[j])
            {
                furthestPath    = i;
                furthestVisited = j;
                found           = true;
            }
        }
        if (found) break;
    }

    // If no intersection found just return current path.
    if (furthestPath == -1 || furthestVisited == -1) return npath;

    // Concatenate paths.

    // Adjust beginning of the buffer to include the visited.
    const int req  = nvisited - furthestVisited;
    const int orig = rcMin(furthestPath + 1, npath);
    int size       = rcMax(0, npath - orig);
    if (req + size > maxPath) size = maxPath - req;
    if (size) memmove(path + req, path + orig, size * sizeof(dtPolyRef));

    // Store visited
    for (int i = 0; i < req; ++i) path[i] = visited[(nvisited - 1) - i];

    return req + size;
}

// This function checks if the path has a small U-turn, that is,
// a polygon further in the path is adjacent to the first polygon
// in the path. If that happens, a shortcut is taken.
// This can happen if the target (T) location is at tile boundary,
// and we're (S) approaching it parallel to the tile edge.
// The choice at the vertex can be arbitrary,
//  +---+---+
//  |:::|:::|
//  +-S-+-T-+
//  |:::|   | <-- the step can end up in here, resulting U-turn path.
//  +---+---+
static int fixupShortcuts(dtPolyRef *path, int npath, dtNavMeshQuery *navQuery)
{
    if (npath < 3) return npath;

    // Get connected polygons
    static const int maxNeis = 16;
    dtPolyRef neis[maxNeis];
    int nneis = 0;

    const dtMeshTile *tile = 0;
    const dtPoly *poly     = 0;
    if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(
            path[0], &tile, &poly)))
        return npath;

    for (unsigned int k = poly->firstLink; k != DT_NULL_LINK;
         k              = tile->links[k].next)
    {
        const dtLink *link = &tile->links[k];
        if (link->ref != 0)
        {
            if (nneis < maxNeis) neis[nneis++] = link->ref;
        }
    }

    // If any of the neighbour polygons is within the next few polygons
    // in the path, short cut to that polygon directly.
    static const int maxLookAhead = 6;
    int cut                       = 0;
    for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--)
    {
        for (int j = 0; j < nneis; j++)
        {
            if (path[i] == neis[j])
            {
                cut = i;
                break;
            }
        }
    }
    if (cut > 1)
    {
        int offset = cut - 1;
        npath -= offset;
        for (int i = 1; i < npath; i++) path[i] = path[i + offset];
    }

    return npath;
}

static bool getSteerTarget(dtNavMeshQuery *navQuery, const float *startPos,
                           const float *endPos, const float minTargetDist,
                           const dtPolyRef *path, const int pathSize,
                           float *steerPos, unsigned char &steerPosFlag,
                           dtPolyRef &steerPosRef, float *outPoints = 0,
                           int *outPointCount = 0)
{
    // Find steer target.
    static const int MAX_STEER_POINTS = 3;
    float steerPath[MAX_STEER_POINTS * 3];
    unsigned char steerPathFlags[MAX_STEER_POINTS];
    dtPolyRef steerPathPolys[MAX_STEER_POINTS];
    int nsteerPath = 0;
    navQuery->findStraightPath(startPos, endPos, path, pathSize, steerPath,
                               steerPathFlags, steerPathPolys, &nsteerPath,
                               MAX_STEER_POINTS);
    if (!nsteerPath) return false;

    if (outPoints && outPointCount)
    {
        *outPointCount = nsteerPath;
        for (int i = 0; i < nsteerPath; ++i)
            dtVcopy(&outPoints[i * 3], &steerPath[i * 3]);
    }

    // Find vertex far enough to steer to.
    int ns = 0;
    while (ns < nsteerPath)
    {
        // Stop at Off-Mesh link or when point is further than slop away.
        if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
            || !inRange(&steerPath[ns * 3], startPos, minTargetDist, 1000.0f))
            break;
        ns++;
    }
    // Failed to find good point to steer to.
    if (ns >= nsteerPath) return false;

    dtVcopy(steerPos, &steerPath[ns * 3]);
    steerPos[1]  = startPos[1];
    steerPosFlag = steerPathFlags[ns];
    steerPosRef  = steerPathPolys[ns];

    return true;
}
////////////////////////////////////////////////////////////////////////////////

RecastNavMesh::RecastNavMesh(/* args */)
{
    _nav_mesh  = nullptr;
    _nav_query = nullptr;

    _filter        = default_filter();
    _setting       = default_setting();
    _poly_pick_ext = default_poly_pick_ext();
}

RecastNavMesh::RecastNavMesh(const float *poly_pick_ext,
                             const struct Setting *setting,
                             const class dtQueryFilter *filter)
{
    _nav_mesh  = nullptr;
    _nav_query = nullptr;

    _filter        = filter ? filter : default_filter();
    _setting       = setting ? setting : default_setting();
    _poly_pick_ext = poly_pick_ext ? poly_pick_ext : default_poly_pick_ext();
}

RecastNavMesh::~RecastNavMesh()
{
    if (_nav_mesh)
    {
        dtFreeNavMesh(_nav_mesh);
    }

    _nav_mesh = nullptr;
}

const float *RecastNavMesh::default_poly_pick_ext() const
{
    // default poly pick ext from RecastDemo
    static const float poly_pick_ext[] = {2.f, 4.f, 2.f};

    return poly_pick_ext;
}
const RecastNavMesh::Setting *RecastNavMesh::default_setting() const
{
    static Setting s;
    s.tileSize   = 64;
    s.cellSize   = 0.3;
    s.cellHeight = 0.2;

    s.agentMaxSlope = 45.;
    s.agentHeight   = 2.;
    s.agentMaxClimb = 0.9;
    s.agentRadius   = 0.6;

    s.edgeMaxLen      = 12.;
    s.edgeMaxError    = 1.3;
    s.regionMinSize   = 8.;
    s.regionMergeSize = 20.;
    s.vertsPerPoly    = 6.;

    s.detailSampleDist     = 6.;
    s.detailSampleMaxError = 1.;

    s.partitionType = SAMPLE_PARTITION_WATERSHED;

    return &s;
}

const dtQueryFilter *RecastNavMesh::default_filter() const
{
    static dtQueryFilter filter;

    // default filter from RecastDemo
    filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
    filter.setExcludeFlags(0);
    filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.);
    filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.);
    filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.);
    filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.);
    filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.);
    filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5);

    return &filter;
}

// ported from RecastDemo bool Sample_SoloMesh::handleBuild()
bool RecastNavMesh::raw_build(InputGeom *m_geom, rcContext *m_ctx)
{
    // set variable compatible to original RecastDemo code unchange
    bool m_keepInterResults             = false;
    bool m_filterLowHangingObstacles    = true;
    bool m_filterLedgeSpans             = true;
    bool m_filterWalkableLowHeightSpans = true;
    unsigned char *m_triareas           = nullptr;
    rcCompactHeightfield *m_chf         = nullptr;
    rcContourSet *m_cset                = nullptr;
    int m_partitionType                 = _setting->partitionType;
    rcPolyMesh *m_pmesh                 = nullptr;
    rcPolyMeshDetail *m_dmesh           = nullptr;
    dtNavMesh *m_navMesh                = nullptr;
    dtStatus status                     = DT_SUCCESS;
    rcConfig m_cfg;

    float m_tileSize             = _setting->tileSize;
    float m_cellSize             = _setting->cellSize;
    float m_cellHeight           = _setting->cellHeight;
    float m_agentMaxSlope        = _setting->agentMaxSlope;
    float m_agentHeight          = _setting->agentHeight;
    float m_agentMaxClimb        = _setting->agentMaxClimb;
    float m_agentRadius          = _setting->agentRadius;
    float m_edgeMaxLen           = _setting->edgeMaxLen;
    float m_edgeMaxError         = _setting->edgeMaxError;
    float m_regionMinSize        = _setting->regionMinSize;
    float m_regionMergeSize      = _setting->regionMergeSize;
    float m_vertsPerPoly         = _setting->vertsPerPoly;
    float m_detailSampleDist     = _setting->detailSampleDist;
    float m_detailSampleMaxError = _setting->detailSampleMaxError;

    ////////////////////////////////////////////////////////////////////////////
    ////// original RecastDemo code

    if (!m_geom || !m_geom->getMesh())
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Input mesh is not specified.");
        return false;
    }

    const float *bmin  = m_geom->getNavMeshBoundsMin();
    const float *bmax  = m_geom->getNavMeshBoundsMax();
    const float *verts = m_geom->getMesh()->getVerts();
    const int nverts   = m_geom->getMesh()->getVertCount();
    const int *tris    = m_geom->getMesh()->getTris();
    const int ntris    = m_geom->getMesh()->getTriCount();

    //
    // Step 1. Initialize build config.
    //

    // Init build configuration from GUI
    memset(&m_cfg, 0, sizeof(m_cfg));
    m_cfg.tileSize               = m_tileSize;
    m_cfg.cs                     = m_cellSize;
    m_cfg.ch                     = m_cellHeight;
    m_cfg.walkableSlopeAngle     = m_agentMaxSlope;
    m_cfg.walkableHeight         = (int)ceilf(m_agentHeight / m_cfg.ch);
    m_cfg.walkableClimb          = (int)floorf(m_agentMaxClimb / m_cfg.ch);
    m_cfg.walkableRadius         = (int)ceilf(m_agentRadius / m_cfg.cs);
    m_cfg.maxEdgeLen             = (int)(m_edgeMaxLen / m_cellSize);
    m_cfg.maxSimplificationError = m_edgeMaxError;
    m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize); // Note: area = size*size
    m_cfg.mergeRegionArea =
        (int)rcSqr(m_regionMergeSize); // Note: area = size*size
    m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
    m_cfg.detailSampleDist =
        m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
    m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

    // Set the area where the navigation will be build.
    // Here the bounds of the input mesh are used, but the
    // area could be specified by an user defined box, etc.
    rcVcopy(m_cfg.bmin, bmin);
    rcVcopy(m_cfg.bmax, bmax);
    rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

    // Reset build times gathering.
    m_ctx->resetTimers();

    // Start the build process.
    m_ctx->startTimer(RC_TIMER_TOTAL);

    m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
    m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
    m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f,
               ntris / 1000.0f);

    //
    // Step 2. Rasterize input polygon soup.
    //

    // Allocate voxel heightfield where we rasterize our input data to.
    rcHeightfield *m_solid = rcAllocHeightfield();
    if (!m_solid)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return false;
    }
    if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height,
                             m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Could not create solid heightfield.");
        return false;
    }

    // Allocate array that can hold triangle area types.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    m_triareas = new unsigned char[ntris];
    if (!m_triareas)
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
        return false;
    }

    // Find triangles which are walkable based on their slope and rasterize
    // them. If your input data is multiple meshes, you can transform them here,
    // calculate the are type for each of the meshes and rasterize them.
    memset(m_triareas, 0, ntris * sizeof(unsigned char));
    rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts,
                            tris, ntris, m_triareas);
    if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris,
                              *m_solid, m_cfg.walkableClimb))
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Could not rasterize triangles.");
        return false;
    }

    if (!m_keepInterResults)
    {
        delete[] m_triareas;
        m_triareas = 0;
    }

    //
    // Step 3. Filter walkables surfaces.
    //

    // Once all geoemtry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    if (m_filterLowHangingObstacles)
        rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
    if (m_filterLedgeSpans)
        rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb,
                           *m_solid);
    if (m_filterWalkableLowHeightSpans)
        rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

    //
    // Step 4. Partition walkable surface to simple regions.
    //

    // Compact the heightfield so that it is faster to handle from now on.
    // This will result more cache coherent data as well as the neighbours
    // between walkable cells will be calculated.
    m_chf = rcAllocCompactHeightfield();
    if (!m_chf)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return false;
    }
    if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight,
                                   m_cfg.walkableClimb, *m_solid, *m_chf))
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Could not build compact data.");
        return false;
    }

    if (!m_keepInterResults)
    {
        rcFreeHeightField(m_solid);
        m_solid = 0;
    }

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return false;
    }

    // (Optional) Mark areas.
    const ConvexVolume *vols = m_geom->getConvexVolumes();
    for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
        rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin,
                             vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

    // Partition the heightfield so that we can use simple algorithm later to
    // triangulate the walkable areas. There are 3 martitioning methods, each
    // with some pros and cons: 1) Watershed partitioning
    //   - the classic Recast partitioning
    //   - creates the nicest tessellation
    //   - usually slowest
    //   - partitions the heightfield into nice regions without holes or
    //   overlaps
    //   - the are some corner cases where this method creates produces holes
    //   and overlaps
    //      - holes may appear when a small obstacles is close to large open
    //      area (triangulation can handle this)
    //      - overlaps may occur if you have narrow spiral corridors (i.e
    //      stairs), this make triangulation to fail
    //   * generally the best choice if you precompute the nacmesh, use this if
    //   you have large open areas
    // 2) Monotone partioning
    //   - fastest
    //   - partitions the heightfield into regions without holes and overlaps
    //   (guaranteed)
    //   - creates long thin polygons, which sometimes causes paths with detours
    //   * use this if you want fast navmesh generation
    // 3) Layer partitoining
    //   - quite fast
    //   - partitions the heighfield into non-overlapping regions
    //   - relies on the triangulation code to cope with holes (thus slower than
    //   monotone partitioning)
    //   - produces better triangles than monotone partitioning
    //   - does not have the corner cases of watershed partitioning
    //   - can be slow and create a bit ugly tessellation (still better than
    //   monotone)
    //     if you have large open areas with small obstacles (not a problem if
    //     you use tiles)
    //   * good choice to use for tiled navmesh with medium and small sized
    //   tiles

    if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
    {
        // Prepare for region partitioning, by calculating distance field along the walkable surface.
        if (!rcBuildDistanceField(m_ctx, *m_chf))
        {
            m_ctx->log(RC_LOG_ERROR,
                       "buildNavigation: Could not build distance field.");
            return false;
        }

        // Partition the walkable surface into simple regions without holes.
        if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea,
                            m_cfg.mergeRegionArea))
        {
            m_ctx->log(RC_LOG_ERROR,
                       "buildNavigation: Could not build watershed regions.");
            return false;
        }
    }
    else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
    {
        // Partition the walkable surface into simple regions without holes.
        // Monotone partitioning does not need distancefield.
        if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea,
                                    m_cfg.mergeRegionArea))
        {
            m_ctx->log(RC_LOG_ERROR,
                       "buildNavigation: Could not build monotone regions.");
            return false;
        }
    }
    else // SAMPLE_PARTITION_LAYERS
    {
        // Partition the walkable surface into simple regions without holes.
        if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea))
        {
            m_ctx->log(RC_LOG_ERROR,
                       "buildNavigation: Could not build layer regions.");
            return false;
        }
    }

    //
    // Step 5. Trace and simplify region contours.
    //

    // Create contours.
    m_cset = rcAllocContourSet();
    if (!m_cset)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
        return false;
    }
    if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError,
                         m_cfg.maxEdgeLen, *m_cset))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
        return false;
    }

    //
    // Step 6. Build polygons mesh from contours.
    //

    // Build polygon navmesh from the contours.
    m_pmesh = rcAllocPolyMesh();
    if (!m_pmesh)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
        return false;
    }
    if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Could not triangulate contours.");
        return false;
    }

    //
    // Step 7. Create detail mesh which allows to access approximate height on each polygon.
    //

    m_dmesh = rcAllocPolyMeshDetail();
    if (!m_dmesh)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
        return false;
    }

    if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist,
                               m_cfg.detailSampleMaxError, *m_dmesh))
    {
        m_ctx->log(RC_LOG_ERROR,
                   "buildNavigation: Could not build detail mesh.");
        return false;
    }

    if (!m_keepInterResults)
    {
        rcFreeCompactHeightfield(m_chf);
        m_chf = 0;
        rcFreeContourSet(m_cset);
        m_cset = 0;
    }

    // At this point the navigation mesh data is ready, you can access it from m_pmesh.
    // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

    //
    // (Optional) Step 8. Create Detour data from Recast poly mesh.
    //

    // The GUI may allow more max points per polygon than Detour can handle.
    // Only build the detour navmesh if we do not exceed the limit.
    if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {
        unsigned char *navData = 0;
        int navDataSize        = 0;

        // Update poly flags from areas.
        for (int i = 0; i < m_pmesh->npolys; ++i)
        {
            if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
                m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

            if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND
                || m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS
                || m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
            {
                m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
            {
                m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
            {
                m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }

        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
        params.verts            = m_pmesh->verts;
        params.vertCount        = m_pmesh->nverts;
        params.polys            = m_pmesh->polys;
        params.polyAreas        = m_pmesh->areas;
        params.polyFlags        = m_pmesh->flags;
        params.polyCount        = m_pmesh->npolys;
        params.nvp              = m_pmesh->nvp;
        params.detailMeshes     = m_dmesh->meshes;
        params.detailVerts      = m_dmesh->verts;
        params.detailVertsCount = m_dmesh->nverts;
        params.detailTris       = m_dmesh->tris;
        params.detailTriCount   = m_dmesh->ntris;
        params.offMeshConVerts  = m_geom->getOffMeshConnectionVerts();
        params.offMeshConRad    = m_geom->getOffMeshConnectionRads();
        params.offMeshConDir    = m_geom->getOffMeshConnectionDirs();
        params.offMeshConAreas  = m_geom->getOffMeshConnectionAreas();
        params.offMeshConFlags  = m_geom->getOffMeshConnectionFlags();
        params.offMeshConUserID = m_geom->getOffMeshConnectionId();
        params.offMeshConCount  = m_geom->getOffMeshConnectionCount();
        params.walkableHeight   = m_agentHeight;
        params.walkableRadius   = m_agentRadius;
        params.walkableClimb    = m_agentMaxClimb;
        rcVcopy(params.bmin, m_pmesh->bmin);
        rcVcopy(params.bmax, m_pmesh->bmax);
        params.cs          = m_cfg.cs;
        params.ch          = m_cfg.ch;
        params.buildBvTree = true;

        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        {
            m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
            return false;
        }

        m_navMesh = dtAllocNavMesh();
        if (!m_navMesh)
        {
            dtFree(navData);
            m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
            return false;
        }

        status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
        if (dtStatusFailed(status))
        {
            dtFree(navData);
            m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
            return false;
        }

        //        status = m_navQuery->init(m_navMesh, 2048);
        //        if (dtStatusFailed(status))
        //        {
        //            m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh
        //            query"); return false;
        //        }
    }

    m_ctx->stopTimer(RC_TIMER_TOTAL);

    // Show performance stats.
    // duLogBuildTimes(*ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
    m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons",
               m_pmesh->nverts, m_pmesh->npolys);

    // m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

    _nav_mesh = m_navMesh;
    return true;
}

/**
 * load mesh data pre generated from Recast
 * @param path a mesh data file
 */
bool RecastNavMesh::load(const char *path)
{
    // ported from RecastDemo dtNavMesh* Sample::loadAll(const char* path)
    FILE *fp = fopen(path, "rb");
    if (!fp) return 0;

    // Read header.
    NavMeshSetHeader header;
    size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
    if (readLen != 1)
    {
        fclose(fp);
        return 0;
    }
    if (header.magic != NAVMESHSET_MAGIC)
    {
        fclose(fp);
        return 0;
    }
    if (header.version != NAVMESHSET_VERSION)
    {
        fclose(fp);
        return 0;
    }

    dtNavMesh *mesh = dtAllocNavMesh();
    if (!mesh)
    {
        fclose(fp);
        return 0;
    }
    dtStatus status = mesh->init(&header.params);
    if (dtStatusFailed(status))
    {
        fclose(fp);
        return 0;
    }

    // Read tiles.
    for (int i = 0; i < header.numTiles; ++i)
    {
        NavMeshTileHeader tileHeader;
        readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
        if (readLen != 1)
        {
            fclose(fp);
            return 0;
        }

        if (!tileHeader.tileRef || !tileHeader.dataSize) break;

        unsigned char *data =
            (unsigned char *)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
        if (!data) break;
        memset(data, 0, tileHeader.dataSize);
        readLen = fread(data, tileHeader.dataSize, 1, fp);
        if (readLen != 1)
        {
            dtFree(data);
            fclose(fp);
            return 0;
        }

        mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA,
                      tileHeader.tileRef, 0);
    }

    fclose(fp);

    if (_nav_mesh)
    {
        dtFreeNavMesh(_nav_mesh);
    }
    _nav_mesh = mesh;

    return true;
}

/**
 * generated mesh data from a obj/gset file
 * @param from a obj/gset file
 */
bool RecastNavMesh::build(const char *from)
{
    if (_nav_mesh)
    {
        delete _nav_mesh;
        _nav_mesh = nullptr;
    }

    rcContext m_ctx;
    InputGeom m_geom;

    if (!m_geom.load(&m_ctx, from))
    {
        m_ctx.log(RC_LOG_ERROR,
                  "buildNavigation: Input mesh is not specified.");
        return false;
    }

    return raw_build(&m_geom, &m_ctx);
}

/**
 * save mesh data to file, ported from RecastDemo
 * void Sample::saveAll(const char* path, const dtNavMesh* mesh)
 */
bool RecastNavMesh::save(const char *path)
{
    const dtNavMesh *mesh = _nav_mesh;
    if (!mesh)
    {
        std::cerr << "No mesh data to save" << std::endl;
        return false;
    }

    FILE *fp = fopen(path, "wb");
    if (!fp)
    {
        std::cerr << "Could not open " << path << " for writing" << std::endl;
        return false;
    }

    // Store header.
    NavMeshSetHeader header;
    header.magic    = NAVMESHSET_MAGIC;
    header.version  = NAVMESHSET_VERSION;
    header.numTiles = 0;
    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile *tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;
        header.numTiles++;
    }
    memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
    fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

    // Store tiles.
    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile *tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;

        NavMeshTileHeader tileHeader;
        tileHeader.tileRef  = mesh->getTileRef(tile);
        tileHeader.dataSize = tile->dataSize;
        fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

        fwrite(tile->data, tile->dataSize, 1, fp);
    }

    fclose(fp);

    return true;
}

int RecastNavMesh::smooth(float *m_spos, float *m_epos, unsigned int *m_polys,
                          int m_npolys, unsigned int m_startRef,
                          float *m_smoothPath, int size, float step)
{
    // ported form RecastDemo void NavMeshTesterTool::recalc()
    // setup some variable to keep potaled code unchange
    const dtQueryFilter &m_filter = *_filter;
    const dtNavMesh *m_navMesh    = _nav_mesh;
    dtNavMeshQuery *m_navQuery    = _nav_query;

    // Iterate over the path to find smooth path on the detail mesh surface.
    dtPolyRef polys[MAX_POLYS];
    memcpy(polys, m_polys, sizeof(dtPolyRef) * m_npolys);
    int npolys = m_npolys;

    float iterPos[3], targetPos[3];
    m_navQuery->closestPointOnPoly(m_startRef, m_spos, iterPos, 0);
    m_navQuery->closestPointOnPoly(polys[npolys - 1], m_epos, targetPos, 0);

    int m_nsmoothPath       = 0;
    const int MAX_SMOOTH    = size;
    const float STEP_SIZE   = step;
    static const float SLOP = 0.01f;

    dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos);
    m_nsmoothPath++;

    // Move towards target a small advancement at a time until target reached or
    // when ran out of memory to store the path.
    while (npolys && m_nsmoothPath < MAX_SMOOTH)
    {
        // Find location to steer towards.
        float steerPos[3];
        unsigned char steerPosFlag;
        dtPolyRef steerPosRef;

        if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP, polys, npolys,
                            steerPos, steerPosFlag, steerPosRef))
            break;

        bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
        bool offMeshConnection =
            (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

        // Find movement delta.
        float delta[3], len;
        dtVsub(delta, steerPos, iterPos);
        len = dtMathSqrtf(dtVdot(delta, delta));
        // If the steer target is end of path or off-mesh link, do not move past the location.
        if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
            len = 1;
        else
            len = STEP_SIZE / len;
        float moveTgt[3];
        dtVmad(moveTgt, iterPos, delta, len);

        // Move
        float result[3];
        dtPolyRef visited[16];
        int nvisited = 0;
        m_navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter,
                                     result, visited, &nvisited, 16);

        npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
        npolys = fixupShortcuts(polys, npolys, m_navQuery);

        float h = 0;
        m_navQuery->getPolyHeight(polys[0], result, &h);
        result[1] = h;
        dtVcopy(iterPos, result);

        // Handle end of path and off-mesh links when close enough.
        if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
        {
            // Reached end of path.
            dtVcopy(iterPos, targetPos);
            if (m_nsmoothPath < MAX_SMOOTH)
            {
                dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos);
                m_nsmoothPath++;
            }
            break;
        }
        else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
        {
            // Reached off-mesh connection.
            float startPos[3], endPos[3];

            // Advance the path up to and over the off-mesh connection.
            dtPolyRef prevRef = 0, polyRef = polys[0];
            int npos = 0;
            while (npos < npolys && polyRef != steerPosRef)
            {
                prevRef = polyRef;
                polyRef = polys[npos];
                npos++;
            }
            for (int i = npos; i < npolys; ++i) polys[i - npos] = polys[i];
            npolys -= npos;

            // Handle the connection.
            dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(
                prevRef, polyRef, startPos, endPos);
            if (dtStatusSucceed(status))
            {
                if (m_nsmoothPath < MAX_SMOOTH)
                {
                    dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos);
                    m_nsmoothPath++;
                    // Hack to make the dotted path not visible during off-mesh connection.
                    if (m_nsmoothPath & 1)
                    {
                        dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos);
                        m_nsmoothPath++;
                    }
                }
                // Move position at the other side of the off-mesh link.
                dtVcopy(iterPos, endPos);
                float eh = 0.0f;
                m_navQuery->getPolyHeight(polys[0], iterPos, &eh);
                iterPos[1] = eh;
            }
        }

        // Store results.
        if (m_nsmoothPath < MAX_SMOOTH)
        {
            dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos);
            m_nsmoothPath++;
        }
    }

    return m_nsmoothPath;
}

/**
 * pathfinding(follow)
 * right-handle coordinate, The upper right corner as the origin, x to right
 * @return the actual point size
 */
unsigned int RecastNavMesh::follow(float sx, float sy, float sz, float ex,
                                   float ey, float ez, float *points,
                                   int max_size, int &use_size, float step)
{
    use_size = 0;
    // ported form RecastDemo void NavMeshTesterTool::recalc()
    if (!_nav_mesh) return DT_FAILURE;

    if (!_nav_query)
    {
        _nav_query = dtAllocNavMeshQuery();
        if (!_nav_query->init(_nav_mesh, 2048)) return DT_FAILURE;
    }

    float m_spos[] = {sx, sy, sz};
    float m_epos[] = {ex, ey, ez};

    dtPolyRef m_startRef;
    dtPolyRef m_endRef;
    _nav_query->findNearestPoly(m_spos, _poly_pick_ext, _filter, &m_startRef, 0);
    _nav_query->findNearestPoly(m_epos, _poly_pick_ext, _filter, &m_endRef, 0);
    if (!m_startRef || !m_endRef) return 0;

    int m_npolys = 0;
    dtPolyRef m_polys[MAX_POLYS];
    dtStatus status =
        _nav_query->findPath(m_startRef, m_endRef, m_spos, m_epos, _filter,
                             m_polys, &m_npolys, MAX_POLYS);
    if (dtStatusFailed(status))
    {
        return DT_FAILURE;
    }

    if (!m_npolys) return status;

    use_size = smooth(m_spos, m_epos, m_polys, m_npolys, m_startRef, points,
                      max_size, step);

    return status;
}

bool RecastNavMesh::is_succeed(unsigned int status)
{
    return dtStatusSucceed(status);
}
bool RecastNavMesh::is_partia(unsigned int status)
{
    return dtStatusDetail(status, DT_PARTIAL_RESULT);
}

/**
 * pathfinding(straight)
 * right-handle coordinate, x axis right, y axis up
 * @return status, use is_xx function to check fail.
 */
unsigned int RecastNavMesh::straight(float sx, float sy, float sz, float ex,
                                     float ey, float ez, float *points,
                                     int max_size, int &use_size, int option)
{
    use_size = 0;
    // ported form RecastDemo void NavMeshTesterTool::recalc()
    if (!_nav_mesh) return DT_FAILURE;

    if (!_nav_query)
    {
        _nav_query = dtAllocNavMeshQuery();
        if (!_nav_query->init(_nav_mesh, 2048)) return DT_FAILURE;
    }

    float m_spos[] = {sx, sy, sz};
    float m_epos[] = {ex, ey, ez};

    dtPolyRef m_startRef;
    dtPolyRef m_endRef;
    _nav_query->findNearestPoly(m_spos, _poly_pick_ext, _filter, &m_startRef, 0);
    _nav_query->findNearestPoly(m_epos, _poly_pick_ext, _filter, &m_endRef, 0);
    if (!m_startRef || !m_endRef) return 0;

    int m_npolys = 0;
    dtPolyRef m_polys[MAX_POLYS];
    dtStatus status =
        _nav_query->findPath(m_startRef, m_endRef, m_spos, m_epos, _filter,
                             m_polys, &m_npolys, MAX_POLYS);

    if (!m_npolys) return status;

    // In case of partial path, make sure the end point is clamped to the last polygon.
    float epos[3];
    dtVcopy(epos, m_epos);
    if (m_polys[m_npolys - 1] != m_endRef)
    {
        status = _nav_query->closestPointOnPoly(m_polys[m_npolys - 1], m_epos,
                                                epos, 0);
        if (dtStatusFailed(status)) return status;
    }

    /// those out param unused for now
    ///  @param[out]	straightPathFlags	Flags describing each point. (See:
    ///  #dtStraightPathFlags) [opt]
    ///  @param[out]	straightPathRefs	The reference id of the polygon that
    ///  is being entered at each point. [opt]
    status = _nav_query->findStraightPath(m_spos, epos, m_polys, m_npolys,
                                          points, nullptr, nullptr, &use_size,
                                          max_size, option);

    return status;
}
