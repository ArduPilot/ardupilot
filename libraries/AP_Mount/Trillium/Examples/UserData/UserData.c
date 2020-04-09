#include "OrionPublicPacket.h"
#include "OrionComm.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Incoming and outgoing packet structures. Inco0ming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, OrionUserData_t *pUser);
static int ProcessKeyboard(void);

int main(int argc, char **argv)
{
    OrionUserData_t UserIn = { USER_DATA_PORT_ETHERNET }, UserOut = { USER_DATA_PORT_PRIMARY };

    // Process the command line arguments
    ProcessArgs(argc, argv, &UserOut);

    // Loop forever
    while (1)
    {
        // Record the initial buffer size
        int Byte, Size = UserOut.size;

        // Get any key presses from the user and stuff them in the buffer if there's room
        while ((Byte = ProcessKeyboard()) && (UserOut.size < sizeof(UserOut.data) - 1))
            UserOut.data[UserOut.size++] = (uint8_t)Byte;

        // If there's data in the buffer and we didn't add anything on this iteration
        if ((UserOut.size > 0) && (UserOut.size == Size))
        {
            // Encode and send the packet
            encodeOrionUserDataPacketStructure(&PktOut, &UserOut);
            OrionCommSend(&PktOut);

            // Tell the user how many bytes got sent out
            printf("\nSending packet %d: %d byte%s...\n", UserOut.id, UserOut.size, (UserOut.size == 1) ? "" : "s");

            // Finally, increment the packet ID and empty the buffer by setting its size to zero
            UserOut.id++;
            UserOut.size = 0;
        }

        // Look for any incoming gimbal packets
        while (OrionCommReceive(&PktIn))
        {
            // If we find a user data packet
            if (decodeOrionUserDataPacketStructure(&PktIn, &UserIn))
            {
                // Add a null terminator to the string
                UserIn.data[UserIn.size] = 0;

                // Now print the incoming data to stdout
                printf("Received Packet %d: %s\n", UserIn.id, (char *)UserIn.data);
            }
        }

        // Flush the stdout buffer and sleep for 1/4 second
        fflush(stdout);
        usleep(250000);
    }

    // Done (actually, we'll never get here...)
    return 0;

}// main

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

static void ProcessArgs(int argc, char **argv, OrionUserData_t *pUser)
{
    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // If the user specified a destination port
    if (argc > 1)
    {
        // Pull it off the argument list
        pUser->port = (UserDataPort_t)atoi(argv[1]);
    }

    // Print the passed-in geopoint command info
    printf("Sending User Data to Gimbal COM%d\n", pUser->port);
    fflush(stdout);

}// ProcessArgs

#ifdef _WIN32
# include <conio.h>
#else
# include <termios.h>
#endif // _WIN32

// Look for a keypress from the user
static int ProcessKeyboard(void)
{
#ifdef _WIN32
	return (_kbhit() == 0) ? 0 : _getch();
#else
    struct termios Old, New;
    char c = 0;

    // If we can get the current attributes for stdin
    if (tcgetattr(fileno(stdin), &Old) >= 0)
    {
        // Copy the current attributes into a new structure
        New = Old;

        // Turn off the echo and canonical output and disable blocking
        New.c_lflag &= ~(ICANON | ECHO);
        New.c_cc[VMIN] = New.c_cc[VTIME] = 0;

        // If we can successfully overwrite the current settings
        if (tcsetattr(fileno(stdin), TCSANOW, &New) >= 0)
        {
            // Try reading from stdin
            if (read(fileno(stdin), &c, 1) != 1)
            {
                // If there's some sort of error, clear whatever came out of read()
                c = 0;
            }
        }

        // Finally, revert the stdin settings to what they were before we were called
        tcsetattr(fileno(stdin), TCSANOW, &Old);
    }

    // And last but not least, return the character we read from stdin (or NULL for nothing)
    return c;
#endif // _WIN32

}// ProcessKeyboard
