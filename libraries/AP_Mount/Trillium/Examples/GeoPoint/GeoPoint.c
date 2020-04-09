#include "OrionPublicPacket.h"
#include "earthposition.h"
#include "OrionComm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, double Pos[3], float Vel[3]);
static BOOL ProcessData(void);

int main(int argc, char **argv)
{
    double TargetPosLla[] = { deg2rad(45.7), deg2rad(-121.5), 30.0 };
    float TargetVelNed[] = { 0.0, 0.0, 0.0 };
    int WaitCount = 0;

    // Process the command line arguments
    ProcessArgs(argc, argv, TargetPosLla, TargetVelNed);

    // This is how you form a packet
    encodeGeopointCmdPacket(&PktOut, TargetPosLla[LAT], TargetPosLla[LON], TargetPosLla[ALT], TargetVelNed, 0, geopointNone);

    // Send the packet
    OrionCommSend(&PktOut);

    // Wait for confirmation from the gimbal, or 5 seconds - whichever comes first
    while ((++WaitCount < 50) && (ProcessData() == FALSE)) usleep(100000);

    // If we timed out waiting, tell the user and return an error code
    if (WaitCount >= 50) KillProcess("Gimbal failed to respond", -1);

    // Done
    return 0;

}// main

static BOOL ProcessData(void)
{
    // Loop through any new incoming packets
    while (OrionCommReceive(&PktIn))
    {
        // If this is a response to the command we just sent, return TRUE
        if (PktIn.ID == ORION_PKT_GEOPOINT_CMD)
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

static void ProcessArgs(int argc, char **argv, double Pos[3], float Vel[3])
{
    char Error[80];

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 7: Vel[2] = atof(argv[6]);          // VelD
    case 6: Vel[1] = atof(argv[5]);          // VelE
    case 5: Vel[0] = atof(argv[4]);          // VelN
    case 4: Pos[2] = atof(argv[3]);          // Alt
    case 3: Pos[1] = deg2rad(atof(argv[2])); // Lon
    case 2: Pos[0] = deg2rad(atof(argv[1])); // Lat
    case 1: break;                           // Serial port path

    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [LAT LON ALT] [VN VE VD]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

    // Print the passed-in geopoint command info
    printf("GEOPOINT: Pos = { %.5f, %.5f, %.1f }, Vel = { %.2f, %.2f, %.2f }\n",
           rad2deg(Pos[0]), rad2deg(Pos[1]), Pos[2],
           Vel[0], Vel[1], Vel[2]);
    fflush(stdout);

}// ProcessArgs
