#include "OrionPublicPacket.h"
#include "fielddecode.h"
#include "OrionComm.h"
#include "StreamDecoder.h"
#include "FFmpeg.h"
#include "KlvParser.h"
#include "earthposition.h"
#include "linearalgebra.h"
#include "mathutilities.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <jpeglib.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, OrionNetworkVideo_t *pSettings, char *pVideoUrl, char *pRecordPath);
static int ProcessKeyboard(void);
static void SaveJpeg(uint8_t *pData, const double Lla[NLLA], uint64_t TimeStamp, int Width, int Height, const char *pPath, int Quality);
static void WriteExifData(struct jpeg_compress_struct *pInfo, const double Lla[NLLA], uint64_t TimeStamp);

int main(int argc, char **argv)
{
    uint8_t VideoFrame[1280 * 720 * 3] = { 0 }, MetaData[1024] = { 0 };
    OrionNetworkVideo_t Settings;
    char VideoUrl[32] = "", RecordPath[256] = "";
    int FrameCount = 0;

    // Zero out the video settings packet to set everything to 'no change'
    memset(&Settings, 0, sizeof(Settings));

    // Video port will default to 15004
    Settings.Port = 15004;
    Settings.StreamType = STREAM_TYPE_H264;

    // Process the command line arguments   
    ProcessArgs(argc, argv, &Settings, VideoUrl, RecordPath);

    // Send the network video settings
    encodeOrionNetworkVideoPacketStructure(&PktOut, &Settings);
    OrionCommSend(&PktOut);

    // If we can't open the video stream
    if (StreamOpen(VideoUrl, RecordPath) == 0)
    {
        // Tell the user and get out of here
        printf("Failed to open video at %s\n", VideoUrl);
        KillProcess("", 1);
    }
    else
        printf("Press S to capture a snapshot or Q to quit\n");

    // Loop forever
    while (1)
    {
        // Run the MPEG-TS stream processor
        while (StreamProcess())
        {
            // If we got a new frame/metadata pair, print a little info to the screen
            printf("Captured %5d frames\r", ++FrameCount);
        }

        // Switch on keyboard input (if any)
        switch (ProcessKeyboard())
        {
        case 's':
        case 'S':
        {
            double Lla[NLLA] = { 0, 0, 0 };
            int Width, Height, Size;
            uint64_t TimeStamp;
            char Path[64];

            // If we can read a KLV UAS data packet out of the decoder
            if (StreamGetMetaData(MetaData, &Size, sizeof(MetaData)))
            {
                int Result;

                // Send the new metadata to the KLV parser
                KlvNewData(MetaData, Size);

                // Grab the gimbal's LLA out of the KLV data
                Lla[LAT] = KlvGetValueDouble(KLV_UAS_SENSOR_LAT, &Result);
                Lla[LON] = KlvGetValueDouble(KLV_UAS_SENSOR_LON, &Result);
                Lla[ALT] = KlvGetValueDouble(KLV_UAS_SENSOR_MSL, &Result);

                // Grab the UNIX timestamp as well
                TimeStamp = KlvGetValueUInt(KLV_UAS_TIME_STAMP, &Result);

                // Print the gimbal LLA data to stdout
                printf("\nImage Pos: %11.6lf %11.6lf %7.1lf", degrees(Lla[LAT]), degrees(Lla[LON]), Lla[ALT]);
            }

            // If we can read the current frame out of the decoder
            if (StreamGetVideoFrame(VideoFrame, &Width, &Height, sizeof(VideoFrame)))
            {
                // Create a file path based on the current frame index
                sprintf(Path, "%05d.jpg", FrameCount);

                // Now save the image as a JPEG
                SaveJpeg(VideoFrame, Lla, TimeStamp, Width, Height, Path, 75);

                // Print some confirmation to stdout
                printf("\nSaved file %s\n", Path);
            }

            break;
        }

        case 'q':
        case 'Q':
            KillProcess("Exiting...", 0);
            break;

        default:
            break;
        };

        // Flush the stdout buffer and sleep for 5ms
        fflush(stdout);
        usleep(5000);
    }

    // Done (actually, we'll never get here...)
    return 0;

}// main

static void SaveJpeg(uint8_t *pData, const double Lla[NLLA], uint64_t TimeStamp, int Width, int Height, const char *pPath, int Quality)
{
    FILE *pFile;

    // If we manage to open the file we're trying to write to
    if ((pFile = fopen(pPath, "wb")) != NULL)
    {
        struct jpeg_compress_struct Info;
        struct jpeg_error_mgr Error;
        unsigned int i;

        // Not sure why this has to happen first...
        Info.err = jpeg_std_error(&Error);

        // Initialize the compressor subsystem
        jpeg_create_compress(&Info);
        jpeg_stdio_dest(&Info, pFile);

        // Populate some information regarding the image format
        Info.image_width      = Width;
        Info.image_height     = Height;
        Info.input_components = 3;
        Info.in_color_space   = JCS_RGB;

        // Now initialize all the internal stuff to defaults
        jpeg_set_defaults(&Info);

        // It's definitely called fastest for a reason... May want to disable this, though, for quality's sake
        Info.dct_method = JDCT_FASTEST;

        // Set quality and get to compressin'
        jpeg_set_quality(&Info, Quality, 1);
        jpeg_start_compress(&Info, 1);

        // Write the EXIF data (if any)
        WriteExifData(&Info, Lla, TimeStamp);

        // Allocate a scanline array
        JSAMPARRAY pScanLines = (JSAMPARRAY)malloc(Height * sizeof(JSAMPROW));

        // For each scanline in the image
        for (i = 0; i < Height; i++)
        {
            // Point this scanline row to the appropriate row in the input data
            pScanLines[i] = &pData[i * Info.image_width * Info.input_components];
        }

        // Write the JPEG data to disk
        jpeg_write_scanlines(&Info, pScanLines, Height);

        // Free the scanline array
        free(pScanLines);

        // Finally, finish and close the file
        jpeg_finish_compress(&Info);
        jpeg_destroy_compress(&Info);
        fclose(pFile);
    }

}// SaveJpeg

const char *LatLonToString(char *pBuffer, double Radians, char SuffixPos, char SuffixNeg)
{
    // Convert from lat/lon to unsigned degrees
    double Degrees = fabs(degrees(Radians));

    // Split into integer and fractional parts
    double Integer = (int)Degrees, Fraction = Degrees - Integer;

    // Finally, format the data as per the XMP spec
    sprintf(pBuffer, "%.0lf,%.6lf%c", Integer, Fraction * 60.0, (Radians < 0) ? SuffixNeg : SuffixPos);

    // Now return a pointer to the buffer that the user passed in
    return pBuffer;

}// LatLonToString

static void WriteExifData(struct jpeg_compress_struct *pInfo, const double Lla[NLLA], uint64_t TimeStamp)
{
    // Convert from UNIX microseconds to GPS milliseconds
    uint64_t GpsTime = TimeStamp / 1000 + (LEAP_SECONDS * 1000) - 315964800000ULL;
    uint32_t Week = GpsTime / 604800000ULL, Itow = GpsTime - Week * 604800000ULL;
    uint8_t Month, Day, Hour, Minute, Second;
    uint16_t Year;

    // Now get date and time from the reconstructed GPS time
    computeDateAndTimeFromWeekAndItow(Week, Itow, LEAP_SECONDS, &Year, &Month, &Day, &Hour, &Minute, &Second);

    // Check for valid GPS time
    if (Year > 2012)
    {
        char Exif[4096], Buffer[64];
        int i = 0;

        // XML header garbage
        i += sprintf(&Exif[i], "http://ns.adobe.com/xap/1.0/");
        Exif[i++] = 0;
        i += sprintf(&Exif[i], "<?xpacket begin='\xef\xbb\xbf' id='W5M0MpCehiHzreSzNTczkc9d'?>\n");
        i += sprintf(&Exif[i], "<x:xmpmeta xmlns:x='adobe:ns:meta/' x:xmptk='XMP Core 5.4.0'>\n");
        i += sprintf(&Exif[i], "<rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>\n\n");
        i += sprintf(&Exif[i], " <rdf:Description rdf:about='' xmlns:exif='http://ns.adobe.com/exif/1.0/'>\n");

        // GPS LLA camera position
        i += sprintf(&Exif[i], "  <exif:GPSLatitude>%s</exif:GPSLatitude>\n", LatLonToString(Buffer, Lla[LAT], 'N', 'S'));
        i += sprintf(&Exif[i], "  <exif:GPSLongitude>%s</exif:GPSLongitude>\n", LatLonToString(Buffer, Lla[LON], 'E', 'W'));
        i += sprintf(&Exif[i], "  <exif:GPSAltitude>%.1lf</exif:GPSAltitude>\n", Lla[ALT]);

        // GPS date/time
        i += sprintf(&Exif[i],"  <exif:GPSTimeStamp>%u:%02u:%02u %02u:%02u:%02u</exif:GPSTimeStamp>\n", Year, Month, Day, Hour, Minute, Second);

        // XML footer garbage
        i += sprintf(&Exif[i], " </rdf:Description>\n");
        i += sprintf(&Exif[i], "</rdf:RDF>\n");
        i += sprintf(&Exif[i], "</x:xmpmeta>\n");

        // Now write the data to the JPEG file
        jpeg_write_marker(pInfo, 0xe1, (const uint8_t *)Exif, i);        
    }

}// WriteExifData

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Kill the video stream parser/recorder
    StreamClose();

    // Close down the active file descriptors
    OrionCommClose();

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static void ProcessArgs(int argc, char **argv, OrionNetworkVideo_t *pSettings, char *pVideoUrl, char *pRecordPath)
{
    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    switch (argc)
    {
    // Recording file path
    case 4: strncpy(pRecordPath, argv[3], 256);
    // Video destination port
    case 3: pSettings->Port = atoi(argv[2]);
    // Video destination IP
    case 2:
    {
        uint8_t Octets[4];

        if (sscanf(argv[1], "%3hhu.%3hhu.%3hhu.%3hhu", &Octets[0], &Octets[1], &Octets[2], &Octets[3]))
        {
            int Index = 0;
            pSettings->DestIp = uint32FromBeBytes(Octets, &Index);
            sprintf(pVideoUrl, "udp://%s:%d", argv[1], pSettings->Port);
        }
        break;
    }
    default:
        printf("USAGE: %s [/path/to/serial | gimbal_ip] video_ip [port] [record_file.ts]\n", argv[0]);
        KillProcess("", 1);
        break;
    };

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
