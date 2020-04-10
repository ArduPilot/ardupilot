#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "OrionPublicPacket.h"
#include "linearalgebra.h"
#include "earthposition.h"
#include "OrionComm.h"

static int GetPathData(OrionPath_t *pPath);
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, OrionPath_t *pPath);

static OrionPkt_t PktIn, PktOut;

int main(int argc, char **argv)
{
    OrionPath_t Path = { 0 };

    // Process the command line arguments
    ProcessArgs(argc, argv, &Path);

    // Grab the path points (up to 15) from path.csv
    if (GetPathData(&Path))
    {
        // Now encode and send the path to the gimbal
        encodeOrionPathPacketStructure(&PktOut, &Path);
        OrionCommSend(&PktOut);

        // Now just loop forever, looking for packets
        while (1)
        {
            // Pull any pending packets off the comm port
            while (OrionCommReceive(&PktIn))
            {
                GeolocateTelemetryCore_t Geo;

                // If this is a valid geolocate telemetry packet
                if (decodeGeolocateTelemetryCorePacketStructure(&PktIn, &Geo))
                {
                    // Print the current path segment information
                    printf("Path segment: from point %2d to point %2d (%3.0f%%), stare time = %.1f\r", Geo.pathFrom, Geo.pathTo, Geo.pathProgress * 100.0f, Geo.stareTime);
                }

            }

            // Give the rest of the system 20ms to work
            fflush(stdout);
            usleep(20000);
        }
    }
    // Path.numPoints is zero for some reason
    else
        KillProcess("Failed to read path data from path.csv", -1);

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

static void ProcessArgs(int argc, char **argv, OrionPath_t *pPath)
{
    char Error[80];

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 4: pPath->crossTrackStepRatio = atof(argv[3]);
    case 3: pPath->numCrossTrackSteps  = atoi(argv[2]);
    case 2: pPath->alongTrackStepAngle = radiansf(atof(argv[1]));
    case 1: break;                           // Serial port path
    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [AlongStepAngle] [CrossSteps] [CrossStepRatio]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

}// ProcessArgs

static int GetPathData(OrionPath_t *pPath)
{
    FILE *pFile = fopen("path.csv", "r");

    // If the file can actually be opened
    if (pFile != NULL)
    {
        double Ecef[NECEF], Lla[NLLA];
        char Buffer[64];

        // Read as many lines out as possible
        while (!feof(pFile) && fgets(Buffer, sizeof(Buffer), pFile))
        {
            // If this line seems to contain an LLA point and we haven't filled this path segment yet
            if ((fscanf(pFile, "%lf, %lf, %lf", &Lla[LAT], &Lla[LON], &Lla[ALT]) == 3) && (pPath->numPoints < MAX_PATH_POINTS))
            {
                // Grab a pointer to the current path point and increment the point count
                Point_t *pPoint = &pPath->Point[pPath->numPoints++];

                // Convert from LLA to ECEF, then down-cast the vector to single-precision
                llaToECEF(Lla, Ecef);
                vector3Convert(Ecef, pPoint->posEcef);

                // Print the point index and location out for the user
                printf("POINT %2d: %10.6lf, %11.6lf, %.0lf\n", pPath->numPoints, Lla[LAT], Lla[LON], Lla[ALT]);
            }
        }

        // Close down the path file
        fclose(pFile);
    }

    // Return the number of points we got from the file
    return pPath->numPoints;

}// GetPathData
