#ifndef FFMPEG_H
#define FFMPEG_H

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/rational.h"
#include "libavutil/avstring.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)

#define av_frame_unref(x)
#define av_frame_get_buffer get_video_buffer
#define av_frame_copy frame_copy_video

#define AV_PIX_FMT_RGB24 PIX_FMT_RGB24

int avformat_alloc_output_context2(AVFormatContext **avctx, AVOutputFormat *oformat,
                                   const char *format, const char *filename);

AVFrame *av_frame_alloc(void);
void av_frame_free(AVFrame **frame);

int frame_copy_video(AVFrame *dst, const AVFrame *src);
int get_video_buffer(AVFrame *frame, int align);

#endif // LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)

#endif //  FFMPEG_H
