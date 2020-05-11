#include "OrionPublicPacket.h"
#include "OrionComm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Incoming and outgoing packet structures. Inco0ming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, double Pos[3], float Vel[3], float *pHeading);
static BOOL ProcessData(void);

int main(int argc, char **argv)
{
    // Heading and estimated heading noise in radians
    float Heading = deg2radf(270.0f), HeadingNoise = deg2radf(3.0f);
    int WaitCount = 0;
    GpsData_t Gps = { 0 };

    // Default latitude, longitude, and altitude. Note that lat/lon are double-precision
    //   values in radians
    Gps.Latitude  = deg2rad(45.7);
    Gps.Longitude = deg2rad(-121.5);
    Gps.Altitude = 300.0;

    // GPS velocity in the NED frame in meters per second
    Gps.VelNED[0] = 3.0f;
    Gps.VelNED[1] = 22.0f;
    Gps.VelNED[2] = -4.0f;

    // GPS diagnostic information
    Gps.PDOP = 2.2f;
    Gps.TrackedSats = 8;
    Gps.Hacc = 10.0f;
    Gps.Vacc = 20.0f;

    // If there's no valid fix, these two parameters should be set to zero
    Gps.FixType = 3;
    Gps.FixState = 1;

    // Process the command line arguments
    ProcessArgs(argc, argv, &Gps.Latitude, Gps.VelNED, &Heading);

    // Form a GPS data packet
    encodeGpsDataPacketStructure(&PktOut, &Gps);

    // Send the packet
    OrionCommSend(&PktOut);

    // Wait for confirmation from the gimbal, or 5 seconds - whichever comes first
    while ((++WaitCount < 50) && (ProcessData() == FALSE)) usleep(100000);

    // If we timed out waiting, tell the user and return an error code
    if (WaitCount >= 50)
        KillProcess("Gimbal failed to respond", -1);
    else
        WaitCount = 0;

    // Now form an external heading packet
    encodeOrionExtHeadingDataPacket(&PktOut, Heading, HeadingNoise, 0, 0, 0);

    // Send the packet
    OrionCommSend(&PktOut);

    // Wait for confirmation from the gimbal, or 5 seconds - whichever comes first
    while ((++WaitCount < 50) && (ProcessData() == FALSE)) usleep(100000);

    // If we timed out waiting, tell the user and return an error code
    if (WaitCount >= 50)
        KillProcess("Gimbal failed to respond", -1);
    else
        WaitCount = 0;

    // Done
    return 0;

}// main

static BOOL ProcessData(void)
{
    // Loop through any new incoming packets
    while (OrionCommReceive(&PktIn))
    {
        // If this is a response to the command we just sent, return TRUE
        if (PktIn.ID == PktOut.ID)
            return TRUE;
    }

    return FALSE;

}// ProcessData

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

static void ProcessArgs(int argc, char **argv, double Pos[3], float Vel[3], float *pHeading)
{
    char Error[80];

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 8: *pHeading = deg2radf(atof(argv[7]));// Heading
    case 7: Vel[2] = atof(argv[6]);             // VelD
    case 6: Vel[1] = atof(argv[5]);             // VelE
    case 5: Vel[0] = atof(argv[4]);             // VelN
    case 4: Pos[2] = atof(argv[3]);             // Alt
    case 3: Pos[1] = deg2rad(atof(argv[2]));    // Lon
    case 2: Pos[0] = deg2rad(atof(argv[1]));    // Lat
    case 1: break;                              // Serial port path

    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [LAT LON ALT] [VEL_N VEL_E VEL_D] [HDG]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

    // Print the passed-in geopoint command info
    printf("LLA: { %.5f, %.5f, %.1f }\nVEL: { %.2f, %.2f, %.2f }\nHDG: %.0f\n",
           rad2deg(Pos[0]), rad2deg(Pos[1]), Pos[2],
           Vel[0], Vel[1], Vel[2],
           rad2degf(*pHeading));
    fflush(stdout);

}// ProcessArgs
