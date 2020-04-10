#include "OrionPublicPacket.h"
#include "earthposition.h"
#include "OrionComm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv);
static BOOL ProcessData(void);

int main(int argc, char **argv)
{
    int WaitCount = 0;

    // Process the command line arguments
    ProcessArgs(argc, argv);

    // Request the camera settings packet
    MakeOrionPacket(&PktOut, getOrionCamerasPacketID(), 0);
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
        // If this is a response to the request we just sent
        if (PktIn.ID == getOrionCamerasPacketID())
        {
            OrionCameras_t Cameras;

            // If the cameras packet decodes properly
            if (decodeOrionCamerasPacketStructure(&PktIn, &Cameras))
            {
                int i;

                // Print a header row to stdout
                printf(" Index  Type     Zoom  WFOV  NFOV\n");
                printf("----------------------------------\n");

                // Loop through each camera in the array
                for (i = 0; i < Cameras.NumCameras; i++)
                {
                    OrionCamSettings_t *pSettings = &Cameras.OrionCamSettings[i];
                    float ArraySize = pSettings->PixelPitch * pSettings->ArrayWidth;
                    float Zoom = 1.0f, Wfov, Nfov;
                    char TypeString[16];

                    // If this camera doesn't exist, skip it
                    if (pSettings->Type == CAMERA_TYPE_NONE)
                        continue;

                    // Build a type string based on the type enumeration
                    switch (pSettings->Type)
                    {
                    case CAMERA_TYPE_VISIBLE: strcpy(TypeString, "Visible"); break;
                    case CAMERA_TYPE_SWIR:    strcpy(TypeString, "SWIR");    break;
                    case CAMERA_TYPE_MWIR:    strcpy(TypeString, "MWIR");    break;
                    case CAMERA_TYPE_LWIR:    strcpy(TypeString, "LWIR");    break;
                    default:                  strcpy(TypeString, "Unknown"); break;
                    }

                    // Calculate max zoom ratio for use in OrionCameraState, avoiding (unlikely) divide by zero
                    if (pSettings->MinFocalLength > 0)
                        Zoom = pSettings->MaxFocalLength / pSettings->MinFocalLength;

                    // Compute wide and narrow horizontal FOV in radians
                    Wfov = atan2f(0.5f * ArraySize, pSettings->MinFocalLength) * 2.0f;
                    Nfov = atan2f(0.5f * ArraySize, pSettings->MaxFocalLength) * 2.0f;

                    // Print the index, type, max zoom, and min/max FOV in degrees for this camera
                    printf(" %5d  %-7s %5.1f %5.1f %5.1f\n", i, TypeString, Zoom, degreesf(Wfov), degreesf(Nfov));
                }

                // Packet decoded: Mission accomplished
                return TRUE;
            }
        }
    }

    // Haven't gotten the response we're looking for yet
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

static void ProcessArgs(int argc, char **argv)
{
    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

}// ProcessArgs
