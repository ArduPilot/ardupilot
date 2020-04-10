#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "LineOfSight.h"
#include "GeolocateTelemetry.h"
#include "OrionComm.h"

#define TILE_CACHE 25
// #define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT printf
#else
# define DEBUG_PRINT(...)
#endif

static int TriangleContainsPoint(const double A[NLLA], const double B[NLLA], const double C[NLLA], double P[NLLA]);
static int GetTile(const TileInfo_t *pTile);
static float GetElevation(double TargetLat, double TargetLon);
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, int *pLevel);

static OrionPkt_t PktIn, PktOut;
static Tile_t Tiles[TILE_CACHE];
static int TileIndex = 0;

// Elevation tile level of detail - defaults to 12
static int TileLevel = 12;

int main(int argc, char **argv)
{
    // Process the command line arguments
    ProcessArgs(argc, argv, &TileLevel);

    // Loop forever
    while (1)
    {
        GeolocateTelemetry_t Geo;

        // Pull all queued up packets off the comm interface
        while (OrionCommReceive(&PktIn))
        {
            // If this packet is a geolocate telemetry packet
            if (DecodeGeolocateTelemetry(&PktIn, &Geo))
            {
                double TargetLla[NLLA], Range;

                // If we got a valid target position from the gimbal
                if (Geo.slantRange > 0)
                {
                    double TargetEcef[NECEF], LosEcef[NECEF];

                    // Convert the 32-bit line of sight vector to double precision
                    vector3Convertf(Geo.base.losECEF, LosEcef);

                    // Add the line of sight vector to the gimbal position, both are in the ECEF coordinate frame
                    vector3Sum(Geo.posECEF, LosEcef, TargetEcef);

                    // Convert the target position from ECEF to LLA
                    ecefToLLA(TargetEcef, TargetLla);

                    // Grab the slant range for printing
                    Range = Geo.slantRange;
                }
                // As a fallback, try finding an intersection with the WGS-84 ellipsoid
                else if (getTerrainIntersection(&Geo, GetElevation, TargetLla, &Range))
                {
                    // Send the computed slant range data to the gimbal
                    encodeOrionRangeDataPacket(&PktOut, Range, 1000, RANGE_SRC_OTHER);
                    OrionCommSend(&PktOut);
                }
                // If both methods fail
                else
                {
                    // Let the user know that we didn't come up with a valid image location and move on
                    printf("TARGET LLA: %-44s\r", "INVALID");
                    continue;
                }

                // If we got a valid intersection, print it out (note that we convert altitude to MSL)
                printf("TARGET LLA: %10.6lf %11.6lf %6.1lf, RANGE: %6.0lf\r",
                       degrees(TargetLla[LAT]),
                       degrees(TargetLla[LON]),
                       TargetLla[ALT] - Geo.base.geoidUndulation,
                       Range);
            }
        }

        // Sleep for 20 ms so as not to hog the entire CPU
        fflush(stdout);
        usleep(20000);
    }

    // Finally, be done!
    return 0;
}

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Close down the active file descriptors
    OrionCommClose();

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static void ProcessArgs(int argc, char **argv, int *pLevel)
{
    char Error[80];

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 2: *pLevel = atoi(argv[1]);         // Tile level of detail
    case 1: break;                           // Serial port path
    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [LoD]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

}// ProcessArgs

static __inline int TriangleContainsPoint(const double A[NLLA], const double B[NLLA], const double C[NLLA], double P[NLLA])
{
    // Compute dot products
    double Dot00 = (C[0] - A[0]) * (C[0] - A[0]) + (C[1] - A[1]) * (C[1] - A[1]); // dot(AC, AC)
    double Dot01 = (C[0] - A[0]) * (B[0] - A[0]) + (C[1] - A[1]) * (B[1] - A[1]); // dot(AC, AB)
    double Dot02 = (C[0] - A[0]) * (P[0] - A[0]) + (C[1] - A[1]) * (P[1] - A[1]); // dot(AC, AP)
    double Dot11 = (B[0] - A[0]) * (B[0] - A[0]) + (B[1] - A[1]) * (B[1] - A[1]); // dot(AB, AB)
    double Dot12 = (B[0] - A[0]) * (P[0] - A[0]) + (B[1] - A[1]) * (P[1] - A[1]); // dot(AB, AP)

    // Compute barycentric coordinates
    double d = Dot00 * Dot11 - Dot01 * Dot01;
    double u = Dot11 * Dot02 - Dot01 * Dot12;
    double v = Dot00 * Dot12 - Dot01 * Dot02;

    // If the point is inside this triangle
    if ((u >= 0) && (v >= 0) && (u + v < d))
    {
        // Interpolate the altitude and return true
        P[ALT] = (u * C[ALT] + v * B[ALT] + (d - (u + v)) * A[ALT]) / d;
        return 1;
    }
    // Otherwise return false
    else
        return 0;

}// TriangleContainsPoint

static int GetTile(const TileInfo_t *pTileInfo)
{
    char Cmd[64], File[64];
    FILE *pFile;
    int i, MaxIndex;

    for (i = 0; i < MIN(TILE_CACHE, TileIndex); i++)
    {
        if (pTileInfo->Level != Tiles[i].Info.Level)
            continue;
        else if (pTileInfo->X != Tiles[i].Info.X)
            continue;
        else if (pTileInfo->Y != Tiles[i].Info.Y)
            continue;
        else
            return i;
    }

    // Pull the appropriate tile from the server and construct the file name string
    sprintf(Cmd, "./get_tile.sh %d %d %d", pTileInfo->Level, pTileInfo->X, pTileInfo->Y);
    system(Cmd);
    sprintf(File, "cache/%d/%d/%d.terrain", pTileInfo->Level, pTileInfo->X, pTileInfo->Y);

    // Open the terrain file
    pFile = fopen(File, "rb");

    // If the file opens successfully
    if (pFile != NULL)
    {
        Tile_t *pTile = &Tiles[TileIndex % TILE_CACHE];
        double CenterLla[NLLA], Scale = PId / (1 << pTileInfo->Level);
        int16_t U = 0, V = 0, H = 0;
        Header_t Header;

        // Delete all heap-allocated storage
        free(pTile->Triangles.pIndices);
        free(pTile->Vertices.pLla);
        free(pTile->Vertices.pH);
        free(pTile->Vertices.pV);
        free(pTile->Vertices.pU);

        // Read header structure
        fread(&Header, sizeof(Header_t), 1, pFile);

        // Convert the ECEF center point of this tile to LLA
        ecefToLLA((double *)&Header, CenterLla);

        // Read in the vertex count
        fread(&pTile->Vertices.Count, sizeof(uint32_t), 1, pFile);

#ifdef DEBUG
        // Set up constants for degrees-to-meters Taylor series expansion
        static const double m1 = 111132.92, m2 = -559.82, m3 = 1.175, m4 = -0.0023;
        static const double p1 = 111412.84, p2 = -93.5,   p3 = 0.118;

        // Print the tile dimensions in meters, along with the number of points
        DEBUG_PRINT(" Tile width   = %.1fm\n", rad2deg(Scale) * (p1 * cos(CenterLla[LAT])) + (p2 * cos(3 * CenterLla[LAT])) + (p3 * cos(5 * CenterLla[LAT])));
        DEBUG_PRINT(" Tile height  = %.1fm\n", rad2deg(Scale) * (m1 + (m2 * cos(2 * CenterLla[LAT])) + (m3 * cos(4 * CenterLla[LAT])) + (m4 * cos(6 * CenterLla[LAT]))));
        DEBUG_PRINT(" Points       = %d\n", pTile->Vertices.Count);
#endif // DEBUG

        // Allocate space for vertex data (U/V/H as well as LLA)
        pTile->Vertices.pU = (uint16_t *)malloc(pTile->Vertices.Count * sizeof(uint16_t));
        pTile->Vertices.pV = (uint16_t *)malloc(pTile->Vertices.Count * sizeof(uint16_t));
        pTile->Vertices.pH = (uint16_t *)malloc(pTile->Vertices.Count * sizeof(uint16_t));
        pTile->Vertices.pLla = (double *)malloc(pTile->Vertices.Count * 3 * sizeof(double));

        // Now read the U/V/H data from the file
        fread(pTile->Vertices.pU, sizeof(uint16_t) * pTile->Vertices.Count, 1, pFile);
        fread(pTile->Vertices.pV, sizeof(uint16_t) * pTile->Vertices.Count, 1, pFile);
        fread(pTile->Vertices.pH, sizeof(uint16_t) * pTile->Vertices.Count, 1, pFile);

        // For each vertex read from the file
        for (i = 0; i < pTile->Vertices.Count; i++)
        {
            // Decode the zigzag data and add the delta to the running raw U/V/H trackers
            U += (pTile->Vertices.pU[i] >> 1) ^ (-(pTile->Vertices.pU[i] & 1));
            V += (pTile->Vertices.pV[i] >> 1) ^ (-(pTile->Vertices.pV[i] & 1));
            H += (pTile->Vertices.pH[i] >> 1) ^ (-(pTile->Vertices.pH[i] & 1));

            // Compute and store the LLA position of this point
            pTile->Vertices.pLla[i * NLLA + LAT] = CenterLla[LAT] + (V / 32767.0f - 0.5f) * Scale;
            pTile->Vertices.pLla[i * NLLA + LON] = CenterLla[LON] + (U / 32767.0f - 0.5f) * Scale;
            pTile->Vertices.pLla[i * NLLA + ALT] = H * (Header.MaxHeight - Header.MinHeight) / 32767.0f + Header.MinHeight;
        }

        // Now read the number of triangles that follow
        fread(&pTile->Triangles.Count, sizeof(uint32_t), 1, pFile);

        // Allocate space for and read in the triangles' vertex data
        pTile->Triangles.pIndices = (uint16_t *)malloc(pTile->Triangles.Count * 3 * sizeof(uint16_t));
        fread(pTile->Triangles.pIndices, sizeof(uint16_t) * pTile->Triangles.Count * 3, 1, pFile);

        // Close the terrain file now that we've gotten all we need from it
        fclose(pFile);

        // For each vertex in each triangle
        for (i = 0, MaxIndex = 0; i < pTile->Triangles.Count * 3; i++)
        {
            // Decode the "high-water-mark" encoded index of this vertex
            if (pTile->Triangles.pIndices[i] == 0)
                pTile->Triangles.pIndices[i] = MaxIndex++;
            else
                pTile->Triangles.pIndices[i] = MaxIndex - pTile->Triangles.pIndices[i];
        }

        pTile->Info = *pTileInfo;

        // Tell the caller that we got the tile
        return (TileIndex++ % TILE_CACHE);
    }

    // No data available... or something
    return -1;

}// GetTile

static float GetElevation(double TargetLat, double TargetLon)
{
    double TargetLla[NLLA] = { TargetLat, TargetLon, -10000.0 };
    double Scale = PId / (1 << TileLevel);
    TileInfo_t TileInfo;
    int Index;

    // Load up the tile description structure with
    TileInfo.Level = TileLevel;
    TileInfo.X = (TargetLon + PId * 1.0f) / Scale;
    TileInfo.Y = (TargetLat + PId * 0.5f) / Scale;

    Index = GetTile(&TileInfo);

    // If we can get the elevation tile containing this lat/lon
    if (Index >= 0)
    {
        Tile_t *pTile = &Tiles[Index];
        int i;

        // For each triangle in the list
        for (i = 0; i < pTile->Triangles.Count; i++)
        {
            // Get pointers to the LLA data of the three vertices in this triangle
            double *pA = &pTile->Vertices.pLla[pTile->Triangles.pIndices[i * 3 + 0] * NLLA];
            double *pB = &pTile->Vertices.pLla[pTile->Triangles.pIndices[i * 3 + 1] * NLLA];
            double *pC = &pTile->Vertices.pLla[pTile->Triangles.pIndices[i * 3 + 2] * NLLA];

            // If the point we're looking for is contained within this triangle
            if (TriangleContainsPoint(pA, pB, pC, TargetLla))
            {
                // No need to continue searching
                break;
            }
        }
    }

    // Return the elevation of this point (or -10000 on failure)
    return TargetLla[ALT];

}// GetElevation
