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
static void ProcessArgs(int argc, char **argv, OrionCmd_t *pCmd);
static BOOL ProcessData(void);

int main(int argc, char **argv)
{
    OrionCmd_t Cmd = { { 0, 0 } };
    int WaitCount = 0;

    // Process the command line arguments
    ProcessArgs(argc, argv, &Cmd);

    // This is how you form a packet
    encodeOrionCmdPacket(&PktOut, &Cmd);

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
        if (PktIn.ID == ORION_PKT_CMD)
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

static void ProcessArgs(int argc, char **argv, OrionCmd_t *pCmd)
{
    char Error[80];

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // Default mode is rate
    pCmd->Mode = ORION_MODE_RATE;

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 6: pCmd->ImpulseTime = atof(argv[5]);         // Impulse time
    case 5: pCmd->Stabilized = (argv[4][0] == '1');    // Stabilized
    case 4:                                            // Mode
    {
        switch (argv[3][0])
        {
        // Rate mode
        default:
        case 'R':
        case 'r':
            pCmd->Mode = ORION_MODE_RATE;
            break;

        // Position mode
        case 'P':
        case 'p':
            pCmd->Mode = ORION_MODE_POSITION;
            break;

        // Disable
        case 'D':
        case 'd':
            pCmd->Mode = ORION_MODE_DISABLED;
            break;

        // FFC
        case 'F':
        case 'f':
            // Note that we use the 'auto' FFC mode. To FFC at a specific location, change this line:
            pCmd->Mode = ORION_MODE_FFC_AUTO;
            break;
        };
    }
    case 3: pCmd->Target[1] = deg2radf(atof(argv[2])); // Tilt target
    case 2: pCmd->Target[0] = deg2radf(atof(argv[1])); // Pan target
    case 1: break;                                     // Serial port path

    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [PAN TILT] [R|P|D] [STABILIZED] [IMPULSE]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

    // Print the passed-in geopoint command info
    printf("COMMAND: Target = { %.1f, %.1f }, Mode = %d, Stabilized = %d, Impulse = %.1f\n",
           rad2deg(pCmd->Target[0]), rad2deg(pCmd->Target[1]), pCmd->Mode,
           pCmd->Stabilized, pCmd->ImpulseTime);
    fflush(stdout);

}// ProcessArgs
