#include "bebop_autonomy/bebop_video_decoder.h"
#include <stdexcept>
#include <boost/lexical_cast.hpp>

namespace bebop_autonomy
{

const char* VideoDecoder::LOG_TAG = "DEC";

// TODO: Move to util
void VideoDecoder::ThrowOnCondition(const bool cond, const std::string &message)
{
  if (!cond) return;
  throw std::runtime_error(message);
}

VideoDecoder::VideoDecoder()
  : codec_initialized_(false),
    format_ctx_ptr_(NULL),
    codec_ctx_ptr_(NULL),
    codec_ptr_(NULL),
    frame_ptr_(NULL),
    frame_rgb_ptr_(NULL),
    img_convert_ctx_ptr_(NULL),
    input_format_ptr_(NULL),
    av_buffer_(NULL)
{
  ;
}

bool VideoDecoder::InitCodec(const uint32_t width, const uint32_t height)
{
  if (codec_initialized_)
  {
    // TODO: Maybe re-initialize
    return true;
  }

  try
  {
    ThrowOnCondition(width == 0 || height == 0, std::string("Invalid frame size:") +
                     boost::lexical_cast<std::string>(width) + " x " + boost::lexical_cast<std::string>(height));

    // Very first init
    avcodec_register_all();
    av_register_all();

    codec_ptr_ = avcodec_find_decoder(CODEC_ID_H264);
    ThrowOnCondition(codec_ptr_ == NULL, "Codec H264 not found!");

    codec_ctx_ptr_ = avcodec_alloc_context3(codec_ptr_);
    codec_ctx_ptr_->width = width;
    codec_ctx_ptr_->height = height;

    ThrowOnCondition(
          avcodec_open2(codec_ctx_ptr_, codec_ptr_, NULL) < 0,
          "Can not open the decoder!");

    frame_ptr_ = avcodec_alloc_frame();
    frame_rgb_ptr_ = avcodec_alloc_frame();

    ThrowOnCondition(!frame_ptr_ || !frame_rgb_ptr_, "Can not allocate memory for frames!");

    const uint32_t num_bytes = avpicture_get_size(PIX_FMT_RGB24, codec_ctx_ptr_->width, codec_ctx_ptr_->height);
    av_buffer_ = (uint8_t*) av_malloc(num_bytes * sizeof(uint8_t));

    ThrowOnCondition(av_buffer_ == NULL,
                     std::string("Can not allocate memory for the buffer: ") +
                     boost::lexical_cast<std::string>(num_bytes));

    ThrowOnCondition(0 == avpicture_fill(
                       (AVPicture*) frame_rgb_ptr_, av_buffer_, PIX_FMT_RGB24,
                       codec_ctx_ptr_->width, codec_ctx_ptr_->height),
                     "Failed to initialize the picture data structure.");
  }
  catch (const std::runtime_error& e)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "%s", e.what());
    Cleanup();
    return false;
  }

  codec_initialized_ = true;
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "H264 Codec is initialized!");
  return true;
}

void VideoDecoder::Cleanup()
{
  if (codec_ctx_ptr_)
  {
    avcodec_close(codec_ctx_ptr_);
  }

  if (frame_ptr_)
  {
    av_free(frame_ptr_);
  }

  if (frame_rgb_ptr_)
  {
    av_free(frame_rgb_ptr_);
  }

  if (img_convert_ctx_ptr_)
  {
    sws_freeContext(img_convert_ctx_ptr_);
  }

  codec_initialized_ = false;
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Cleaned up!");
}

VideoDecoder::~VideoDecoder()
{
  Cleanup();
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Dstr!");
}

void VideoDecoder::ConvertFrameToRGB()
{
  if (!img_convert_ctx_ptr_)
  {
    img_convert_ctx_ptr_ = sws_getContext(codec_ctx_ptr_->width, codec_ctx_ptr_->height, codec_ctx_ptr_->pix_fmt,
                                          codec_ctx_ptr_->width, codec_ctx_ptr_->height, PIX_FMT_RGB24,
                                          SWS_FAST_BILINEAR, NULL, NULL, NULL);
  }
  sws_scale(img_convert_ctx_ptr_, frame_ptr_->data, frame_ptr_->linesize, 0,
            codec_ctx_ptr_->height, frame_rgb_ptr_->data, frame_rgb_ptr_->linesize);
}

bool VideoDecoder::Decode(const ARCONTROLLER_Frame_t *ar_frame_ptr_)
{
  if (!codec_initialized_)
  {
    if (!InitCodec(ar_frame_ptr_->width, ar_frame_ptr_->height))
    {
      return false;
    }
  }


  // TODO: Optimize this
  AVPacket packet;
  av_init_packet(&packet);
  av_new_packet(&packet, ar_frame_ptr_->used);
  //packet.data = ar_frame_ptr_->data;
  memcpy(packet.data, ar_frame_ptr_->data, ar_frame_ptr_->used);

  int frame_finished = 0;
  int result = avcodec_decode_video2(codec_ctx_ptr_, frame_ptr_, &frame_finished, &packet);

  if (result >=0 && frame_finished > 0)
  {
    ConvertFrameToRGB();
  }

  av_free_packet(&packet);
  return true;
}

}  // namespace bebop_autonomy
