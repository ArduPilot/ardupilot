#include "OrionComm.h"

#if defined(__linux__) || defined(__APPLE__)

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>

static struct sockaddr *GetSockAddr(uint32_t Address, unsigned short port);

static int Handle = -1;

BOOL OrionCommOpenSerial(const char *pPath)
{
    // Open a file descriptor for the serial port
    Handle = open(pPath, O_RDWR | O_NOCTTY | O_NDELAY);

    // If we actually managed to open something
    if (Handle >= 0)
    {
        struct termios Port;

        // Make sure this is a serial port and that we can get its attributes
        if (!isatty(Handle) || tcgetattr(Handle, &Port))
        {
            // If we can't, close and invalidate the file descriptor
            close(Handle);
            Handle = -1;
        } 
        else
        {
            // Otherwise, clear out the port attributes structure
            memset(&Port, 0, sizeof(Port));

            // Now set up all the other miscellaneous flags appropriately
            Port.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

            // Try passing the new attributes to the port
            if (tcsetattr(Handle, TCSANOW, &Port) != 0)
            {
                // If it didn't work, close and invalidate the port
                close(Handle);
                Handle = -1;
            }
        }
    }

    // Tell the user if this failed or, if not, which COM port they're trying to use
    if (Handle == -1)
        printf("Failed to open %s\n", pPath);
    else
        printf("Looking for gimbal on %s...\n", pPath);

    // Return the file descriptor
    return Handle != -1;

}// OrionCommOpenSerial

BOOL OrionCommIpStringValid(const char *pAddress)
{
    uint32_t Address; 

    // Return TRUE if this is a valid IP address
    return inet_pton(AF_INET, pAddress, &Address) == 1;

}// OrionCommIpStringValid

BOOL OrionCommOpenNetworkIp(const char *pAddress)
{
    // Open a new UDP socket for auto-discovery
    int UdpHandle = socket(AF_INET, SOCK_DGRAM, 0);
    uint32_t BroadcastAddr = INADDR_BROADCAST;
    char IpString[INET_ADDRSTRLEN];

    // Try converting the address to a uint32_t
    if (OrionCommIpStringValid(pAddress) == TRUE)
    {
        // Convert the IPaddress string to a 32-bit IPv4 address value
        inet_pton(AF_INET, pAddress, &BroadcastAddr);

        // Now print out the broadcast address we're pinging
        printf("Looking for gimbal on %s...\n", inet_ntop(AF_INET, &BroadcastAddr, IpString, INET_ADDRSTRLEN));

        // Roll the bytes for our GetSockAddr function
        BroadcastAddr = ntohl(BroadcastAddr);
    }
    else
    {
        // Close the discovery handle and return a failure
        close(UdpHandle);
        return FALSE;
    }

    // If the socket looks good
    if (UdpHandle >= 0)
    {
        BOOL Broadcast = TRUE;
        int WaitCount = 0;
        char Buffer[64];
        OrionPkt_t Pkt;

        // Bind to the proper port to get responses from the gimbal
        bind(UdpHandle, GetSockAddr(INADDR_ANY, UDP_IN_PORT), sizeof(struct sockaddr_in));

        // Make this socket non blocking
        fcntl(UdpHandle, F_SETFL, O_NONBLOCK);

        // Allow the socket to send packets to the broadcast address
        setsockopt(UdpHandle, SOL_SOCKET, SO_BROADCAST, (char *)&Broadcast, sizeof(BOOL));

        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);

        // Wait for up to 20 iterations
        while (WaitCount++ < 20)
        {
            socklen_t Size = sizeof(struct sockaddr_in);

            // Send a version request packet
            sendto(UdpHandle, (char *)&Pkt, Pkt.Length + ORION_PKT_OVERHEAD, 0, GetSockAddr(BroadcastAddr, UDP_OUT_PORT), sizeof(struct sockaddr_in));

            // If we get data back forom the gimbal
            if (recvfrom(UdpHandle, Buffer, 64, 0, GetSockAddr(INADDR_ANY, UDP_IN_PORT), &Size) > 0)
            {
                // Pull the gimbal's IP address from the datagram header
                UInt32 Address = ntohl(((struct sockaddr_in *)GetSockAddr(0, 0))->sin_addr.s_addr);

                // Open a file descriptor for the TCP comm socket
                Handle = socket(AF_INET, SOCK_STREAM, 0);

                // Bind to the right incoming port
                bind(Handle, GetSockAddr(INADDR_ANY, TCP_PORT), sizeof(struct sockaddr_in));

                // Connect to the gimbal's server socket (note this is a blocking call)
                connect(Handle, GetSockAddr(Address, TCP_PORT), sizeof(struct sockaddr_in));

                // Now make the socket non-blocking for future reads/writes
                fcntl(Handle, F_SETFL, O_NONBLOCK);

                // Convert the IP address to network byte order
                Address = htonl(Address);

                // Now print out the IP address that we connected to and break out of the loop
                printf("Connected to %s\n", inet_ntop(AF_INET, &Address, IpString, INET_ADDRSTRLEN));
                break;
            }

            // Sleep for 1/10th of a second
            usleep(100000);
        }

        // If we timed out waiting for a response, let the user know
        if (WaitCount >= 20)
        {
            // Broadcast address byte roll, part one million
            BroadcastAddr = htonl(BroadcastAddr);

            // Let the user know we failed to connect
            printf("Failed to connect to %s\n", inet_ntop(AF_INET, &BroadcastAddr, IpString, INET_ADDRSTRLEN));
        }

        // Close the UDP handle down now that we're done with it
        close(UdpHandle);
    }

    // Return a possibly valid handle to this socket
    return Handle != -1;

}// OrionCommOpenNetworkIp

void OrionCommClose(void)
{
    // Easy enough, just close the file descriptor
    close(Handle);

}// OrionCommClose

BOOL OrionCommSend(const OrionPkt_t *pPkt)
{
    // Write the packet, including header data, to the file descriptor
    return write(Handle, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD) > 0;

}// OrionCommSend

BOOL OrionCommReceive(OrionPkt_t *pPkt)
{
    static OrionPkt_t Pkt = { 0 };
    UInt8 Buffer;

    // As long as we keep getting bytes, keep reading them in one by one
    while (read(Handle, (char *)&Buffer, 1) == 1)
    {
        // If this byte is the end of a valid packet
        if (LookForOrionPacketInByte(&Pkt, Buffer))
        {
            // Copy the packet into the passed-in location and return a success
            *pPkt = Pkt;
            return TRUE;
        }
    }

    // Nope, no packets yet
    return FALSE;

}// OrionCommReceive

// Quickly and easily constructs a sockaddr pointer for a bunch of different functions.
//   Call this function with Address == Port == 0 to access the pointer, or pass in
//   actual values to construct a new sockaddr.
static struct sockaddr *GetSockAddr(uint32_t Address, unsigned short Port)
{
    static struct sockaddr_in SockAddr;

    // If address and port are both zero, don't modify the structure
    if (Address || Port)
    {
        // Otherwise, populate it with the requested IP address and port
        memset(&SockAddr, 0, sizeof(SockAddr));
        SockAddr.sin_family = AF_INET;
        SockAddr.sin_addr.s_addr = htonl(Address);
        SockAddr.sin_port = htons(Port);
    }

    // Return a casted pointer to the sockaddr_in structure
    return (struct sockaddr *)&SockAddr;

}// GetSockAddr
#endif // __linux__
