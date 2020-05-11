#include "StreamDecoder.h"
#include "Constants.h"
#include "FFmpeg.h"

#include <string.h>

static AVFormatContext *pInputContext = NULL;
static AVFormatContext *pOutputContext = NULL;
static AVCodecContext *pCodecContext = NULL;
static AVFrame *pFrame = NULL;
static AVFrame *pFrameCopy = NULL;
static AVPacket Packet;

static uint8_t *pMetaData = NULL;
static uint64_t MetaDataBufferSize = 0;
static uint64_t MetaDataSize = 0;
static uint64_t MetaDataBytes = 0;

static int VideoStream = 0;
static int DataStream = 0;

int StreamOpen(const char *pUrl, const char *pRecordPath)
{
    AVCodec *pCodec;

    // FFmpeg startup stuff
    avcodec_register_all();
    av_register_all();
    avformat_network_init();
    av_log_set_level(AV_LOG_QUIET);

    // Allocate a new format context
    pInputContext = avformat_alloc_context();

    // Have avformat_open_input timeout after 5s
    AVDictionary *pOptions = 0;
    av_dict_set(&pOptions, "timeout", "5000000", 0);

    // If the stream doesn't open
    if (avformat_open_input(&pInputContext, pUrl, NULL, &pOptions) < 0)
    {
        // Clean up the allocated resources (if any...) and exit with a failure code
        StreamClose();
        return 0;
    }

    // If there don't appear to be an valid streams in the transport stream
    if (pInputContext->nb_streams == 0)
    {
        // Clean up the allocated resources (if any...) and exit with a failure code
        StreamClose();
        return 0;
    }

    // Get the stream indices for video and metadata
    VideoStream = av_find_best_stream(pInputContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    DataStream  = av_find_best_stream(pInputContext, AVMEDIA_TYPE_DATA,  -1, -1, NULL, 0);

    // Set the format context to playing
    av_read_play(pInputContext);

    // Get a codec pointer based on the video stream's codec ID and allocate a context
    pCodec = avcodec_find_decoder(pInputContext->streams[VideoStream]->codec->codec_id);
    pCodecContext = avcodec_alloc_context3(pCodec);

    // Open the newly allocated codec context
    avcodec_open2(pCodecContext, pCodec, NULL);

    // If the user passed in a record path
    if (pRecordPath && strlen(pRecordPath))
    {
        int i;

        // Pull any additional stream information out of the file
        //   NOTE: Older FFmpeg/libav builds may hang indefinitely here!
        avformat_find_stream_info(pInputContext, NULL);

        // Allocate a format context for the output file
        avformat_alloc_output_context2(&pOutputContext, NULL, NULL, pRecordPath);

        // For each stream in the UDP stream
        for (i = 0; i < pInputContext->nb_streams; i++)
        {
            // Mirror this stream to the output format context
            AVStream *pStream = avformat_new_stream(pOutputContext, pInputContext->streams[i]->codec->codec);
            avcodec_copy_context(pStream->codec, pInputContext->streams[i]->codec);

            // Add a stream header if the output format calls for it
            if (pOutputContext->oformat->flags & AVFMT_GLOBALHEADER)
                pStream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;
        }

        // Open the record file and write the header out
        avio_open(&pOutputContext->pb, pRecordPath, AVIO_FLAG_WRITE);
        avformat_write_header(pOutputContext, NULL);
    }

    // Allocate the decode and output frame structures
    pFrame = av_frame_alloc();
    pFrameCopy = av_frame_alloc();

    // Finally, initialize the AVPacket structure
    av_init_packet(&Packet);
    Packet.data = NULL;
    Packet.size = 0;

    // Done - return 1 to indicate success
    return 1;

}// StreamOpen

void StreamClose(void)
{
    // Free the AVFrames
    av_frame_free(&pFrame);
    av_frame_free(&pFrameCopy);

    // If we allocated a codec context
    if (pCodecContext)
    {
        // Close it then free it
        avcodec_close(pCodecContext);
        av_free(pCodecContext);
    }

    // If we allocated a format context
    if (pInputContext)
    {
        // Pause, close and free it
        av_read_pause(pInputContext);
        avformat_close_input(&pInputContext);
        avformat_free_context(pInputContext);
    }

    // If there's an output context open
    if (pOutputContext)
    {
        // Write a trailer to the file and close it out
        av_write_trailer(pOutputContext);
        avio_close(pOutputContext->pb);
        avformat_free_context(pOutputContext);
    }

    // Nullify all the pointers we just messed with
    pCodecContext  = NULL;
    pFrame         = NULL;
    pInputContext = NULL;
    pOutputContext  = NULL;

}// StreamClose

int StreamProcess(void)
{
    // New video/metadata flags - note that NewMetaData == 1 if it's not in the TS
    int NewVideo = 0, NewMetaData = (pInputContext->nb_streams < 2);

    // As long as we can keep reading packets from the UDP socket
    while (av_read_frame(pInputContext, &Packet) >= 0)
    {
        int Index = Packet.stream_index;

        // If this packet belongs to the video stream
        if (Index == VideoStream)
        {
            // Pass it to the h.264 decoder
            avcodec_decode_video2(pCodecContext, pFrame, &NewVideo, &Packet);

            // If this packet finished a video frame
            if (NewVideo)
            {
                // If the incoming frame size doesn't match the output frame
                if ((pFrame->width  != pFrameCopy->width) ||
                    (pFrame->height != pFrameCopy->height))
                {
                    // Free the output frame and allocate a new one
                    av_frame_free(&pFrameCopy);
                    pFrameCopy = av_frame_alloc();

                    // Copy all the metadata (width, height, etc.) over
                    //   NOTE: av_frame_copy_props *should* do this but doesn't seem to...
                    memcpy(pFrameCopy, pFrame, sizeof(AVFrame));

                    // Now allocate a data buffer associated with this frame
                    av_frame_get_buffer(pFrameCopy, 0);
                }

                // Copy the image data from the decoder frame to the output frame
                av_frame_copy(pFrameCopy, pFrame);

                // Finally, unref the frame and free the packet
                av_frame_unref(pFrame);
            }
        }
        // If this is the KLV stream data
        else if (Index == DataStream)
        {
            // If we have a full metadata packet in memory, zero out the size and index
            if (MetaDataBytes == MetaDataSize)
                MetaDataBytes = MetaDataSize = 0;

            // If we don't have any metadata buffered up yet and this packet is big enough for a US key and size
            if ((MetaDataBytes == 0) && (Packet.size > 17))
            {
                // UAS LS universal key
                static const uint8_t KlvHeader[16] = {
                    0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01,
                    0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00
                };

                // Try finding the KLV header in this packet
                const uint8_t *pStart = memmem(Packet.data, Packet.size, KlvHeader, 16);
                const uint8_t *pSize = pStart + 16;

                // If we found the header and the size tag is contained in this packet
                if ((pStart != 0) && ((pSize - Packet.data) < Packet.size))
                {
                    // Initialize the header size to US key + 1 size byte and zero KLV tag bytes
                    uint64_t KlvSize = 0, HeaderSize = 17;

                    // If the size is a multi-byte BER-OID size
                    if (pSize[0] & 0x80)
                    {
                        // Get the size of the size (up to )
                        int Bytes = pSize[0] & 0x07, i;

                        // If the entire size field is contained in this packet
                        if (&pSize[Bytes] < &Packet.data[Packet.size])
                        {
                            // Build the size up from the individual bytes
                            for (i = 0; i < Bytes; i++)
                                KlvSize = (KlvSize << 8) | pSize[i + 1];
                        }

                        // Add the additional size bytes to the header size
                        HeaderSize += Bytes;
                    }
                    // Otherwise, just read the size byte straight out of byte 16
                    else
                        KlvSize = pSize[0];

                    // If we got a valid local set size
                    if (KlvSize > 0)
                    {
                        // Compute the maximum bytes to copy out of the packet
                        int MaxBytes = Packet.size - (pStart - Packet.data);
                        int TotalSize = HeaderSize + KlvSize;
                        int BytesToCopy = MIN(MaxBytes, TotalSize);

                        // If our local buffer is too small for the incoming data
                        if (MetaDataBufferSize < TotalSize)
                        {
                            // Reallocate enough space and store the new buffer size
                            pMetaData = (uint8_t *)realloc(pMetaData, TotalSize);
                            MetaDataBufferSize = TotalSize;
                        }

                        // Now copy the new data into the start of the local buffer
                        memcpy(pMetaData, pStart, BytesToCopy);
                        MetaDataSize = TotalSize;
                        MetaDataBytes = BytesToCopy;
                    }
                }
            }
            // Otherwise, if we're mid-packet
            else if (MetaDataBytes < MetaDataSize)
            {
                // Figure out the number of bytes to copy out of this particular packet
                int BytesToCopy = MIN(Packet.size, MetaDataSize - MetaDataBytes);

                // Copy into the local buffer in the right spot and increment the index
                memcpy(&pMetaData[MetaDataBytes], Packet.data, BytesToCopy);
                MetaDataBytes += BytesToCopy;
            }

            // There's new metadata if the size is non-zero and equal to the number of bytes read in
            NewMetaData = (MetaDataSize != 0) && (MetaDataBytes == MetaDataSize);
        }

        // If we have an open output file
        if (pOutputContext)
        {
            static int64_t StartPts = 0, StartDts = 0;
            static int64_t LastPts = -1, LastDts = -1;

            // If we just jumped into the middle of the stream or its timestamps have reset
            if ((LastPts < 0) || (Packet.pts < LastPts))
            {
                // Create an adjustment parameter for both PTS and DTS
                StartPts += (Packet.pts - LastPts) - 1;
                StartDts += (Packet.dts - LastDts) - 1;
            }

            // Save the current PTS/DTS for detecting discontinuities
            LastPts = Packet.pts;
            LastDts = Packet.dts;

            // Adjust the PTS/DTS values to create a continuous stream
            Packet.pts -= StartPts;
            Packet.dts -= StartDts;

            // Rescale all the PTS/DTS nonsense
            Packet.pts = av_rescale_q(Packet.pts, pOutputContext->streams[Index]->time_base, pInputContext->streams[Index]->time_base);
            Packet.dts = av_rescale_q(Packet.dts, pOutputContext->streams[Index]->time_base, pInputContext->streams[Index]->time_base);
            Packet.duration = av_rescale_q(Packet.duration, pOutputContext->streams[Index]->time_base, pInputContext->streams[Index]->time_base);
            Packet.pos = -1;

            // Write the frame to the file
            av_interleaved_write_frame(pOutputContext, &Packet);
        }

        // Free the packet data
        av_free_packet(&Packet);

        // Return 1 if both a video frame and KLV packet have been read in
        if (NewVideo && NewMetaData)
            return 1;
    }

    // No new data if we made it here
    return 0;

}// StreamProcess

int StreamGetVideoFrame(uint8_t *pFrameData, int *pWidth, int *pHeight, int MaxBytes)
{
    // If we actually have frame data to copy out and it won't overrun pFrameData
    if ((pFrameCopy->width > 0) && (pFrameCopy->height > 0) && ((pFrame->width * pFrame->height * 3) < MaxBytes))
    {
        // Allocate a context for colorspace conversion and do some data marshaling 
        struct SwsContext *pContext  = sws_getContext(pFrameCopy->width, pFrameCopy->height, pFrameCopy->format,
                                                      pFrameCopy->width, pFrameCopy->height, AV_PIX_FMT_RGB24,
                                                      SWS_FAST_BILINEAR, NULL, NULL, NULL);
        uint8_t *pData[3] = { pFrameData, NULL, NULL };
        int Stride[3] = { pFrameCopy->width * 3, 0, 0 };

        // Copy the frame width/height out to the caller
        *pWidth  = pFrameCopy->width;
        *pHeight = pFrameCopy->height;

        // Now do the actual colorspace conversion
        sws_scale(pContext, (const uint8_t **)pFrameCopy->data, pFrameCopy->linesize, 0, pFrameCopy->height, pData, Stride);
        sws_freeContext(pContext);

        // Tell the caller this function succeeded
        return 1;
    }

    // No such data
    return 0;

}// StreamGetVideoFrame

int StreamGetMetaData(uint8_t *pData, int *pBytes, int MaxBytes)
{
    // If there's a valid metadata buffer and we're not going to overrun pData
    if ((pMetaData != 0) && (MetaDataBytes < MaxBytes))
    {
        // Copy the buffered KLV data into pData and send out the buffer size
        memcpy(pData, pMetaData, MetaDataBytes);
        *pBytes = MetaDataBytes;

        // A return value of 1 signifies that that pData contains a whole KLV UAS data packet
        return 1;
    }

    // Either the caller needs to allocate more space or we haven't seen any metadata yet
    return 0;

}// StreamGetMetaData

