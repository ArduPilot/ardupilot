#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H

#include "earthposition.h"

typedef struct
{
    // Tile center ECEF position
    double Center[NECEF];

    // Minimum and maximum height in this tile
    float MinHeight;
    float MaxHeight;

    // Bounding sphere ECEF center and radius
    double Sphere[NECEF];
    double SphereRadius;

    // Horizon something or other...
    double Horizon[NECEF];
} Header_t;

typedef struct
{
    // Number of vertices
    uint32_t Count;

    // North/east position in tile
    uint16_t *pU;
    uint16_t *pV;

    // Height of vertex
    uint16_t *pH;

    // LLA position of vertex
    double *pLla;
} Vertices_t;

typedef struct
{
    // Number of triangles
    uint32_t Count;

    // List of triangle vertex index triples
    uint16_t *pIndices;
} Triangles_t;

typedef struct
{
    // Tile level of detail
    int Level;

    // X index
    int X;

    // Y index
    int Y;
} TileInfo_t;

typedef struct
{
    TileInfo_t Info;
    Vertices_t Vertices;
    Triangles_t Triangles;
} Tile_t;

#endif // LINEOFSIGHT_H
