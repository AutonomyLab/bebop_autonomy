/**
Software License Agreement (BSD)

\file      bebop_video_decoder.h
\authors   Mani Monajjemi <mmonajje@sfu.ca>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef BEBOP_AUTONOMY_BEBOP_VIDEO_DECODER_H
#define BEBOP_AUTONOMY_BEBOP_VIDEO_DECODER_H

extern "C"
{
  #include "libARController/ARCONTROLLER_Error.h"
  #include "libARController/ARCONTROLLER_Frame.h"
  #include <libavcodec/avcodec.h>
  #include <libavformat/avformat.h>
  #include <libavformat/avio.h>
  #include <libswscale/swscale.h>
}

#include <string>
#include <vector>

// https://github.com/libav/libav/commit/104e10fb426f903ba9157fdbfe30292d0e4c3d72
// https://github.com/libav/libav/blob/33d18982fa03feb061c8f744a4f0a9175c1f63ab/doc/APIchanges#L697
#if (LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54, 25, 0))
#define AV_CODEC_ID_H264 CODEC_ID_H264
#endif

// https://github.com/libav/libav/blob/33d18982fa03feb061c8f744a4f0a9175c1f63ab/doc/APIchanges#L653
#if (LIBAVCODEC_VERSION_INT < AV_VERSION_INT(51, 42, 0))
#define AV_PIX_FMT_YUV420P PIX_FMT_YUV420P
#endif

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#define av_frame_free avcodec_free_frame
#endif

namespace bebop_driver
{

class VideoDecoder
{
private:
  static const char* LOG_TAG;

  bool codec_initialized_;
  bool first_iframe_recv_;
  AVFormatContext* format_ctx_ptr_;
  AVCodecContext* codec_ctx_ptr_;
  AVCodec* codec_ptr_;
  AVFrame* frame_ptr_;
  AVFrame* frame_rgb_ptr_;
  AVPacket packet_;
  SwsContext* img_convert_ctx_ptr_;
  AVInputFormat* input_format_ptr_;
  uint8_t *frame_rgb_raw_ptr_;

  bool update_codec_params_;
  std::vector<uint8_t> codec_data_;

  static void ThrowOnCondition(const bool cond, const std::string& message);
  bool InitCodec();
  bool ReallocateBuffers();
  void CleanupBuffers();
  void Reset();

  void ConvertFrameToRGB();

public:
  VideoDecoder();
  ~VideoDecoder();

  bool SetH264Params(uint8_t* sps_buffer_ptr, uint32_t sps_buffer_size,
                     uint8_t* pps_buffer_ptr, uint32_t pps_buffer_size);
  bool Decode(const ARCONTROLLER_Frame_t* bebop_frame_ptr_);
  inline uint32_t GetFrameWidth() const {return codec_initialized_ ? codec_ctx_ptr_->width : 0;}
  inline uint32_t GetFrameHeight() const {return codec_initialized_ ? codec_ctx_ptr_->height : 0;}

  inline const uint8_t* GetFrameRGBRawCstPtr() const {return frame_rgb_raw_ptr_;}
};

}  // namespace bebop_driver

#endif  // BEBOP_AUTONOMY_BEBOP_VIDEO_DECODER_H
