#include "OrionComm.h"

#ifdef _WIN32

#include <winsock2.h>
#include <stdio.h>
#include <ws2tcpip.h>

static HANDLE SerialHandle = INVALID_HANDLE_VALUE;
static SOCKET TcpSocket = INVALID_SOCKET;

static struct sockaddr *GetSockAddr(uint32_t Address, unsigned short Port);

BOOL OrionCommOpenSerial(const char *pPath)
{
	// Declare variables and structures
    SerialHandle = CreateFileA(pPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // If the handle is valid
    if (SerialHandle != INVALID_HANDLE_VALUE)
    {
        DCB Params = { sizeof(DCB) };
        COMMTIMEOUTS Timeouts;

        // Try to get the current state
        if (GetCommState(SerialHandle, &Params) == FALSE)
        {
            // If that failed, close the handle and invalidate it
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }

        // Otherwise, fill in the fields we care about
        Params.BaudRate = CBR_115200;
        Params.ByteSize = 8;
        Params.StopBits = ONESTOPBIT;
        Params.Parity   = NOPARITY;

        // Try changing the serial port settings
        if (SetCommState(SerialHandle, &Params) == FALSE)
        {
            // Close and invalidate the handle
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }

        // If getting the existing timeout values fails
        if (GetCommTimeouts(SerialHandle, &Timeouts) == FALSE)
        {
            // Close and invalidate the handle
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }

        // Set the timeouts for non-blocking mode
        Timeouts.ReadIntervalTimeout = MAXDWORD;
        Timeouts.ReadTotalTimeoutConstant = 0;
        Timeouts.ReadTotalTimeoutMultiplier = 0;

        // Try passing the new timeout struct
        if (SetCommTimeouts(SerialHandle, &Timeouts) == FALSE)
        {
            // Close and invalidate the handle on failure
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }
    }

    // Tell the user if this failed or, if not, which COM port they're trying to use
    if (SerialHandle == INVALID_HANDLE_VALUE)
        printf("Failed to open %s\n", pPath);
    else
        printf("Looking for gimbal on %s...\n", pPath);

    // Return false
    return SerialHandle != INVALID_HANDLE_VALUE;

}// OrionCommOpenSerial

BOOL OrionCommIpStringValid(const char *pAddress)
{
    uint32_t Address = inet_addr(pAddress);

    // Return TRUE if this is a valid IP address
    return (Address != INADDR_ANY) && (Address != INADDR_NONE);

}// OrionCommIpStringValid

BOOL OrionCommOpenNetworkIp(const char *pAddress)
{
    // Open a new UDP socket for auto-discovery
    WSADATA WsaData;
    WSAStartup(MAKEWORD(2, 0), &WsaData);
    SOCKET UdpHandle = socket(AF_INET, SOCK_DGRAM, 0);
    uint32_t BroadcastAddr;

    // If we were passed a valid IP string
    if (OrionCommIpStringValid(pAddress) == TRUE)
    {
        // Convert the IP address string to a 32-bit IPv4 address value
        BroadcastAddr = inet_addr(pAddress);

        // Now print out the broadcast address we're pinging
        printf("Looking for gimbal on %s...\n", inet_ntoa(((struct sockaddr_in *)GetSockAddr(BroadcastAddr, UDP_OUT_PORT))->sin_addr));
        
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
    if (UdpHandle != INVALID_SOCKET)
    {
        BOOL Broadcast = TRUE;
        int WaitCount = 0;
        char Buffer[64];
        OrionPkt_t Pkt;
        u_long Arg = 1;

        // Bind to the proper port to get responses from the gimbal
        bind(UdpHandle, GetSockAddr(INADDR_ANY, UDP_IN_PORT), sizeof(struct sockaddr_in));

        // Make this socket non blocking
        ioctlsocket(UdpHandle, FIONBIO, &Arg);

        // Allow the socket to send packets to the broadcast address
        setsockopt(UdpHandle, SOL_SOCKET, SO_BROADCAST, (char *)&Broadcast, sizeof(BOOL));

        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);

        // Wait for up to 20 iterations
        while (WaitCount++ < 20)
        {
            int Size = sizeof(struct sockaddr_in);

            // Send a version request packet
            sendto(UdpHandle, (char *)&Pkt, Pkt.Length + ORION_PKT_OVERHEAD, 0, GetSockAddr(BroadcastAddr, UDP_OUT_PORT), sizeof(struct sockaddr_in));

            // If we get data back forom the gimbal
            if (recvfrom(UdpHandle, Buffer, 64, 0, GetSockAddr(INADDR_ANY, UDP_IN_PORT), &Size) > 0)
            {
                // Pull the gimbal's IP address from the datagram header
                UInt32 Address = ntohl(((struct sockaddr_in *)GetSockAddr(0, 0))->sin_addr.s_addr);

                // Open a file descriptor for the TCP comm socket
                TcpSocket = socket(AF_INET, SOCK_STREAM, 0);

                // Bind to the right incoming port
                bind(TcpSocket, GetSockAddr(INADDR_ANY, TCP_PORT), sizeof(struct sockaddr_in));

                // Connect to the gimbal's server socket (note this is a blocking call)
                connect(TcpSocket, GetSockAddr(Address, TCP_PORT), sizeof(struct sockaddr_in));

                // Now make the socket non-blocking for future reads/writes
                ioctlsocket(TcpSocket, FIONBIO, &Arg);

                // Now print out the IP address that we connected to and break out of the loop
                printf("Connected to %s\n", inet_ntoa(((struct sockaddr_in *)GetSockAddr(0, 0))->sin_addr));
                break;
            }

            // Sleep for 1/10th of a second
            Sleep(100);
        }

        // If we timed out waiting for a response, let the user know
        if (WaitCount >= 20)
        {
            // Broadcast address byte roll, part one million
            BroadcastAddr = htonl(BroadcastAddr);

            // Let the user know we failed to connect
            printf("Failed to connect to %s\n", inet_ntoa(((struct sockaddr_in *)GetSockAddr(BroadcastAddr, UDP_OUT_PORT))->sin_addr));
        }

        // Close the UDP socket now that we're done with it
        closesocket(UdpHandle);
    }

    // Return a possibly valid handle to this socket
    return TcpSocket != INVALID_SOCKET;

}// OrionCommOpenNetworkIp

void OrionCommClose(void)
{
    // Easy enough, just close the file descriptor
    CloseHandle(SerialHandle);
    closesocket(TcpSocket);

}// OrionCommClose

BOOL OrionCommSend(const OrionPkt_t *pPkt)
{
    DWORD Bytes;

    // Write the packet, including header data, to the file descriptor
    if (SerialHandle != INVALID_HANDLE_VALUE)
        return WriteFile(SerialHandle, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD, &Bytes, NULL);
    else
        return send(TcpSocket, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD, 0) != SOCKET_ERROR;

}// OrionCommSend

BOOL OrionCommReceive(OrionPkt_t *pPkt)
{
    static OrionPkt_t Pkt = { 0 };
    UInt8 Byte;

    if (SerialHandle != INVALID_HANDLE_VALUE)
    {
        COMSTAT Status;

        // As long as there are bytes to be read out of the receive queue
        while (ClearCommError(SerialHandle, NULL, &Status) && (Status.cbInQue > 0))
        {
            DWORD BytesRead = 0;

            // Read a byte
            ReadFile(SerialHandle, &Byte, 1, &BytesRead, NULL);

            // If this byte is the end of a valid packet
            if ((BytesRead == 1) && (LookForOrionPacketInByte(&Pkt, Byte) == TRUE))
            {
                // Copy the packet into the passed-in location and return a success
                *pPkt = Pkt;
                return TRUE;
            }
            // Otherwise, if some sort of error occurred
            else if (BytesRead != 1)
            {
                // Close and invalidate the serial port
                CloseHandle(SerialHandle);
                SerialHandle = INVALID_HANDLE_VALUE;
            }
        }
    }
    else
    {
        // As long as we keep getting bytes, keep reading them in one by one
        while (recv(TcpSocket, (char *)&Byte, 1, 0) == 1)
        {
            // If this byte is the end of a valid packet
            if (LookForOrionPacketInByte(&Pkt, Byte))
            {
                // Copy the packet into the passed-in location and return a success
                *pPkt = Pkt;
                return TRUE;
            }
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
#endif // _WIN32
