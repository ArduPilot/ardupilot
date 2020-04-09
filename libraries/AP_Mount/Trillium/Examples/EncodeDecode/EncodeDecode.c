#include "OrionPublicPacketShim.h"
#include <string.h>
#include <stdio.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// Process a chunk of incoming data from a bytestream
static void ProcessData(const UInt8 *pData, UInt32 Length);

int main(int argc, char **argv)
{
    UInt8 Buffer[64];
    OrionCmd_t Cmd;

    // Form a command that will tell the gimbal to move at 10 deg/s in pan for one second
    Cmd.Target[GIMBAL_AXIS_PAN]  = deg2radf(10.0f);
    Cmd.Target[GIMBAL_AXIS_TILT] = deg2radf(0.0f);
    Cmd.Mode = ORION_MODE_RATE;
    Cmd.ImpulseTime = 1.0f;
    Cmd.Stabilized = FALSE;

    // IMPORTANT: Zero out the packet structure before using for decoding
    memset(&PktIn, 0, sizeof(PktIn));

    // This is how you form a packet
    encodeOrionCmdPacket(&PktOut, &Cmd);

    // For now, I fake sending it by copying it straight into the input buffer
    memcpy(Buffer, &PktOut, PktOut.Length + ORION_PKT_OVERHEAD);

    // Process the "incoming data"
    ProcessData(Buffer, PktOut.Length + ORION_PKT_OVERHEAD);

    // Print a trailing line feed so the prompt doesn't overwrite the data
    printf("\n");

    // Done
    return 0;

}// main

void ProcessData(const UInt8  *pData, UInt32 Length)
{
    UInt32 i;

    // For each byte in the receive buffer
    for (i = 0; i < Length; i++)
    {
        // If this byte terminates a valid packet
        if (LookForOrionPacketInByte(&PktIn, pData[i]))
        {
            // Decide what to do based on packet ID
            switch (PktIn.ID)
            {
            // Encoder position/velocity report
            case ORION_PKT_CMD:
            {
                OrionCmd_t Cmd;

                // If this encoder data packet contains valid data
                if (decodeOrionCmdPacketStructure(&PktIn, &Cmd))
                {
                    // Print the command data to stdout
                    printf("Pan: %.1f deg/s, Tilt: %.1f deg/s, ImpulseTime: %.1f, Mode: %d, Stabilized: %d\r",
                           rad2degf(Cmd.Target[GIMBAL_AXIS_PAN]),
                           rad2degf(Cmd.Target[GIMBAL_AXIS_TILT]),
                           Cmd.ImpulseTime, Cmd.Mode, Cmd.Stabilized);
                }
                break;
            }

            // Other packets: TBD
            default:
                break;
            };
        }
    }

}// ProcessData
