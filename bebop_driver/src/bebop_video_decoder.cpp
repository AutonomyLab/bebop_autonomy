/**
Software License Agreement (BSD)

\file      bebop_video_decoder.cpp
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
#include "bebop_driver/bebop_video_decoder.h"
#include "bebop_driver/metadata_struct.h"
#include <netinet/in.h>

#include <stdexcept>
#include <algorithm>
#include <string>
#include <iostream>

#include <inttypes.h>
#include <bitset>

#include <boost/lexical_cast.hpp>

extern "C"
{
  #include "libARSAL/ARSAL_Print.h"
}

namespace bebop_driver
{
  StreamingMetadataV1Extended_t metadata;
  StreamingMetadataV1Extended_t metadata_temp;

const char* VideoDecoder::LOG_TAG = "Decoder";

// TODO(mani-monaj): Move to util, inline
void VideoDecoder::ThrowOnCondition(const bool cond, const std::string &message)
{
  if (!cond) return;
  throw std::runtime_error(message);
}

VideoDecoder::VideoDecoder()
  : codec_initialized_(false),
    first_iframe_recv_(false),
    format_ctx_ptr_(NULL),
    codec_ctx_ptr_(NULL),
    codec_ptr_(NULL),
    frame_ptr_(NULL),
    frame_rgb_ptr_(NULL),
    img_convert_ctx_ptr_(NULL),
    input_format_ptr_(NULL),
    frame_rgb_raw_ptr_(NULL),
    update_codec_params_(false)
{}


bool VideoDecoder::InitCodec()
{
  if (codec_initialized_)
  {
    return true;
  }

  try
  {
    // Very first init
    avcodec_register_all();
    av_register_all();
    av_log_set_level(AV_LOG_QUIET);

    codec_ptr_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    ThrowOnCondition(codec_ptr_ == NULL, "Codec H264 not found!");

    codec_ctx_ptr_ = avcodec_alloc_context3(codec_ptr_);
    codec_ctx_ptr_->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx_ptr_->skip_frame = AVDISCARD_DEFAULT;
    codec_ctx_ptr_->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
    codec_ctx_ptr_->skip_loop_filter = AVDISCARD_DEFAULT;
    codec_ctx_ptr_->workaround_bugs = AVMEDIA_TYPE_VIDEO;
    codec_ctx_ptr_->codec_id = AV_CODEC_ID_H264;
    codec_ctx_ptr_->skip_idct = AVDISCARD_DEFAULT;
    // At the beginning we have no idea about the frame size
    codec_ctx_ptr_->width = 0;
    codec_ctx_ptr_->height = 0;

    if (codec_ptr_->capabilities & CODEC_CAP_TRUNCATED)
    {
      codec_ctx_ptr_->flags |= CODEC_FLAG_TRUNCATED;
    }
    codec_ctx_ptr_->flags2 |= CODEC_FLAG2_CHUNKS;

    frame_ptr_ = av_frame_alloc();
    ThrowOnCondition(!frame_ptr_ , "Can not allocate memory for frames!");

    ThrowOnCondition(
          avcodec_open2(codec_ctx_ptr_, codec_ptr_, NULL) < 0,
          "Can not open the decoder!");

    av_init_packet(&packet_);
  }
  catch (const std::runtime_error& e)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "%s", e.what());
    Reset();
    return false;
  }

  codec_initialized_ = true;
  first_iframe_recv_ = false;
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "H264 Codec is partially initialized!");
  return true;
}

bool VideoDecoder::ReallocateBuffers()
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Buffer reallocation request");
  if (!codec_initialized_)
  {
    return false;
  }

  try
  {
    ThrowOnCondition(codec_ctx_ptr_->width == 0 || codec_ctx_ptr_->width == 0,
                     std::string("Invalid frame size:") +
                     boost::lexical_cast<std::string>(codec_ctx_ptr_->width) +
                     " x " + boost::lexical_cast<std::string>(codec_ctx_ptr_->width));

    const uint32_t num_bytes = avpicture_get_size(PIX_FMT_RGB24, codec_ctx_ptr_->width, codec_ctx_ptr_->width);
    frame_rgb_ptr_ = av_frame_alloc();

    ThrowOnCondition(!frame_rgb_ptr_, "Can not allocate memory for frames!");

    frame_rgb_raw_ptr_ = reinterpret_cast<uint8_t*>(av_malloc(num_bytes * sizeof(uint8_t)));
    ThrowOnCondition(frame_rgb_raw_ptr_ == NULL,
                     std::string("Can not allocate memory for the buffer: ") +
                     boost::lexical_cast<std::string>(num_bytes));
    ThrowOnCondition(0 == avpicture_fill(
                       reinterpret_cast<AVPicture*>(frame_rgb_ptr_), frame_rgb_raw_ptr_, PIX_FMT_RGB24,
                       codec_ctx_ptr_->width, codec_ctx_ptr_->height),
                     "Failed to initialize the picture data structure.");

    img_convert_ctx_ptr_ = sws_getContext(codec_ctx_ptr_->width, codec_ctx_ptr_->height, codec_ctx_ptr_->pix_fmt,
                                          codec_ctx_ptr_->width, codec_ctx_ptr_->height, PIX_FMT_RGB24,
                                          SWS_FAST_BILINEAR, NULL, NULL, NULL);
  }
  catch (const std::runtime_error& e)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "%s", e.what());
    Reset();  // reset() is intentional
    return false;
  }

  return true;
}

void VideoDecoder::CleanupBuffers()
{
  if (frame_rgb_ptr_)
  {
    av_free(frame_rgb_ptr_);
  }

  if (frame_rgb_raw_ptr_)
  {
    av_free(frame_rgb_raw_ptr_);
  }

  if (img_convert_ctx_ptr_)
  {
    sws_freeContext(img_convert_ctx_ptr_);
  }

  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Buffer cleanup!");
}

void VideoDecoder::Reset()
{
  if (codec_ctx_ptr_)
  {
    avcodec_close(codec_ctx_ptr_);
  }

  if (frame_ptr_)
  {
    av_free(frame_ptr_);
  }

  CleanupBuffers();

  codec_initialized_ = false;
  first_iframe_recv_ = false;
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Reset!");
}

VideoDecoder::~VideoDecoder()
{
  Reset();
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Dstr!");
}

void VideoDecoder::ConvertFrameToRGB()
{
  if (!codec_ctx_ptr_->width || !codec_ctx_ptr_->height) return;
  sws_scale(img_convert_ctx_ptr_, frame_ptr_->data, frame_ptr_->linesize, 0,
            codec_ctx_ptr_->height, frame_rgb_ptr_->data, frame_rgb_ptr_->linesize);
}

bool VideoDecoder::SetH264Params(uint8_t *sps_buffer_ptr, uint32_t sps_buffer_size,
                                 uint8_t *pps_buffer_ptr, uint32_t pps_buffer_size)
{
  // This function is called in the same thread as Decode(), so no sync is necessary
  // TODO: Exact sizes + more error checkings
  update_codec_params_ = (sps_buffer_ptr && pps_buffer_ptr &&
                          sps_buffer_size && pps_buffer_size &&
                          (pps_buffer_size < 32) && (sps_buffer_size < 32));

  if (update_codec_params_)
  {
    codec_data_.resize(sps_buffer_size + pps_buffer_size);
    std::copy(sps_buffer_ptr, sps_buffer_ptr + sps_buffer_size, codec_data_.begin());
    std::copy(pps_buffer_ptr, pps_buffer_ptr + pps_buffer_size, codec_data_.begin() + sps_buffer_size);
  }
  else
  {
    // invalid data
    codec_data_.clear();
  }

  return update_codec_params_;
}

bool VideoDecoder::Decode(const ARCONTROLLER_Frame_t *bebop_frame_ptr_)
{
  if (!codec_initialized_)
  {
    if (!InitCodec())
    {
      ARSAL_PRINT(ARSAL_PRINT_WARNING, LOG_TAG, "Codec initialization failed!");
      return false;
    }
  }

  /*
   * For VideoStream2, we trick avcodec whenever we receive a new SPS/PPS
   * info from the Bebop. SetH264Params() function will fill a buffer with SPS/PPS
   * data, then these are passed to avcodec_decode_video2() here, once for each SPS/PPS update.
   * Apparantly, avcodec_decode_video2() function picks up the changes and apply them to
   * upcoming video packets.
   *
   * More info on VS v2.0: http://developer.parrot.com/blog/2016/ARSDK-3-8-release/
   *
   * */
  if (update_codec_params_ && codec_data_.size())
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Updating H264 codec parameters (Buffer Size: %u) ...", codec_data_.size());
    packet_.data = &codec_data_[0];
    packet_.size = codec_data_.size();
    int32_t frame_finished = 0;
    const int32_t len = avcodec_decode_video2(codec_ctx_ptr_, frame_ptr_, &frame_finished, &packet_);
    if (len >= 0 && len == packet_.size)
    {
      // success, skip this step until next codec update
      update_codec_params_ = false;
    }
    else
    {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Unexpected error while updating H264 parameters.");
      return false;
    }
  }

  if (!bebop_frame_ptr_->data || !bebop_frame_ptr_->used)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Invalid frame data. Skipping.");
    return false;
  }

  packet_.data = bebop_frame_ptr_->data;
  packet_.size = bebop_frame_ptr_->used;


  printf("Check for NULL Pointer!!!!!\n");

    if (bebop_frame_ptr_->metadata == NULL)
    {printf("NULL Pointer!!!!!");}

    //metadata_temp = ntohl(metadata);
    //metadata=metadata_temp;

    //printf("Pointer Adress: %p\n",bebop_frame_ptr_->metadata);
    printf("Metadata Size: %d \n",bebop_frame_ptr_->metadataSize);

    memcpy(&metadata,bebop_frame_ptr_->metadata,bebop_frame_ptr_->metadataSize);

    printf("\nStreamingMetadataV1Extended_t:\n");
    printf("specific: \t\t0x%x\n",ntohs(metadata.specific));
    printf("length: \t\t%d\n\n",ntohs(metadata.length));

    // std::bitset<16> spec_bit(metadata.specific);
    // std::bitset<16> len_bit(metadata.length);
    //
    // std::string s1=spec_bit.to_string();
    // std::string s2=len_bit.to_string();
    // std::bitset<32> firstbits(s2 + s1);
    // int bits_int=(int)(firstbits.to_ulong());
    // std::bitset<32> ntohled(ntohl(bits_int));
    // std::cout << "First 32 Bits: " << spec_bit << len_bit << "\nconcatenated: " << firstbits <<  "\n" << ntohled << "\n";

    // std::bitset<32> alt2_bit(ntohl(metadata.alt));
    // std::string alt2_string(alt2_bit.to_string());
    // std::string alt2_m=alt2_string.substr(0,15);
    // std::string alt2_cm=alt2_string.substr(16,31);
    // std::bitset<16> alt2_m_bit(alt2_m);
    // std::bitset<16> alt2_cm_bit(alt2_cm);
    //
    // std::cout << "altitude: " << alt2_m_bit.to_ulong() << "." << alt2_cm_bit.to_ulong() << "m\n"<< alt2_m_bit << alt2_cm_bit << "\n";


    printf("droneYaw: \t\t%d\t",ntohs(metadata.droneYaw));
    std::bitset<16> droneyaw_bit(ntohs(metadata.droneYaw));
    std::cout << droneyaw_bit << "\n";
    printf("dronePitch: \t\t%d\t",ntohs(metadata.dronePitch));
    std::bitset<16> dronepitch_bit(ntohs(metadata.dronePitch));
    std::cout << dronepitch_bit << "\n";

    //Reverse engineered height
    // std::string yaw_string=droneyaw_bit.to_string();
    // std::string pitch_string=dronepitch_bit.to_string();
    // std::string meters=yaw_string;
    // std::string cms=pitch_string;
    // std::bitset mbit=
    //std::cout << "altitude: " << droneyaw_bit.to_ulong() << "." << dronepitch_bit.to_ulong() << "m\n";


    printf("droneRoll: \t\t%d\t",ntohs(metadata.droneRoll));
    std::bitset<16> droneroll_bit(ntohs(metadata.droneRoll));
    std::cout << droneroll_bit << "\n\n";

    printf("cameraPan: \t\t%d\t",VideoDecoder::int16Swap(metadata.cameraPan));
    std::bitset<16> camerapan_bit(VideoDecoder::int16Swap(metadata.cameraPan));
    std::cout << camerapan_bit << "\n";
    printf("cameraTilt: \t\t%d\t",VideoDecoder::int16Swap(metadata.cameraTilt));
    std::bitset<16> cameratilt_bit(VideoDecoder::int16Swap(metadata.cameraTilt));
    std::cout << cameratilt_bit << "\n\n";

    printf("frameW: \t\t%d\t",VideoDecoder::int16Swap(metadata.frameW));
    std::bitset<16> framew_bit(VideoDecoder::int16Swap(metadata.frameW));
    std::cout << framew_bit << "\n";
    printf("frameX: \t\t%d\t",VideoDecoder::int16Swap(metadata.frameX));
    std::bitset<16> framex_bit(VideoDecoder::int16Swap(metadata.frameX));
    std::cout << framex_bit << "\n";
    printf("frameY: \t\t%d\t",VideoDecoder::int16Swap(metadata.frameY));
    std::bitset<16> framey_bit(VideoDecoder::int16Swap(metadata.frameY));
    std::cout << framey_bit << "\n";
    printf("frameZ: \t\t%d\t",VideoDecoder::int16Swap(metadata.frameZ));
    std::bitset<16> framez_bit(VideoDecoder::int16Swap(metadata.frameZ));
    std::cout << framez_bit << "\n\n";

    printf("exposureTime: \t\t%d\n",VideoDecoder::int16Swap(metadata.exposureTime));
    printf("gain: \t\t\t%d\n",VideoDecoder::int16Swap(metadata.gain));

    std::bitset<8> wifi_bit(metadata.wifiRssi);
    std::cout << "wifiRssi (bits): \t\t" << wifi_bit << "\n";
    std::bitset<8> bat_bit(metadata.batteryPercentage);
    std::cout << "batteryPercentage (bits):\t"<< bat_bit << "\n\n";

    std::bitset<32> lat_bit(metadata.gpsLatitude);
    std::cout << "gpsLatitude (bits): \t\t" << lat_bit << "\n";
    std::bitset<32> long_bit(metadata.gpsLongitude);
    std::cout << "gpsLongitude (bits): \t\t" << long_bit << "\n";
    std::bitset<32> gpsalt_bit(metadata.gpsAltitudeAndSv);
    std::cout << "gpsAltitudeAndSv (bits): \t" << gpsalt_bit << "\n";
    std::bitset<32> alt_bit(metadata.altitude);
    std::cout << "altitude (bits): \t\t" << alt_bit << "\n";
    std::bitset<32> home_bit(metadata.distanceFromHome);
    std::cout << "distanceFromHome (bits): \t" << home_bit << "\n\n";


    //printf("\n. . .\n\n");

    printf("xSpeed: \t\t%d\t",VideoDecoder::int16Swap(metadata.xSpeed));
    std::bitset<16> xspeed_bit(VideoDecoder::int16Swap(metadata.xSpeed));
    std::cout << xspeed_bit << "\n";
    printf("ySpeed: \t\t%d\t",VideoDecoder::int16Swap(metadata.ySpeed));
    std::bitset<16> yspeed_bit(VideoDecoder::int16Swap(metadata.ySpeed));
    std::cout << yspeed_bit << "\n";
    printf("zSpeed: \t\t%d\t",VideoDecoder::int16Swap(metadata.zSpeed));
    std::bitset<16> zspeed_bit(VideoDecoder::int16Swap(metadata.zSpeed));
    std::cout << zspeed_bit << "\n\n";

    std::bitset<8> state_bit(metadata.state);
    std::cout << "State (bits): \t\t\t" << state_bit << "\n";
    std::bitset<8> mode_bit(metadata.state);
    std::cout << "Mode (bits): \t\t\t" << mode_bit << "\n\n";
  //  int deadbeef = 0xDEADBEEF;
  //printf("TESTING DEADBEEF %x %x\n", deadbeef, ntohl(deadbeef));


    //printf("\t\t\t.\n\t\t\t.\n\t\t\t.\n");




    // printf("cameraPan: %d\n",VideoDecoder::int16Swap(metadata.cameraPan));
    //printf("cameraPan: %d\n",metadata.cameraPan);
    //std::bitset<16> int_bit(VideoDecoder::int16Swap(metadata.exposureTime));
    //std::bitset<8> int_bit(metadata.batteryPercentage);

    //std::cout << "Drone Yaw: "<< int_bit << "\n";


    // int *test;
    // *test==2;
    // int test2;
    // memcpy(&test2,test,sizeof(int));
    // printf("test2: %d\n ",test2);
  //Debug Print


       //printf("Battery: %" PRIu16 "\n",metadata->batteryPercentage);
    //printf("Battery Percentage: %d\n",metadata->length);

    //printf("droneYaw: %d\n",metadata.droneYaw);
    //printf("Metadata_size: %d\n",sizeof(metadata));
    //printf("Timestamp: %ld\n",bebop_frame_ptr_->);


  const uint32_t width_prev = GetFrameWidth();
  const uint32_t height_prev = GetFrameHeight();

  int32_t frame_finished = 0;
  while (packet_.size > 0)
  {
    const int32_t len = avcodec_decode_video2(codec_ctx_ptr_, frame_ptr_, &frame_finished, &packet_);
    if (len >= 0)
    {
      if (frame_finished)
      {
        if ((GetFrameWidth() != width_prev) || (GetFrameHeight() != height_prev))
        {
          ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Frame size changed to %u x %u", GetFrameWidth(), GetFrameHeight());
          if (!ReallocateBuffers())
          {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Buffer reallocation failed!");
          }
        }
        ConvertFrameToRGB();
      }

      if (packet_.data)
      {
        packet_.size -= len;
        packet_.data += len;
      }
    }
    else
    {
      return false;
    }
  }
  return true;
}



uint16_t VideoDecoder::uint16Swap(uint16_t s)
  {
  	unsigned char b1, b2;
  	b1 = s & 255;
  	b2 = (s >> 8) & 255;
  	return (b1 << 8) + b2;
  }

int16_t VideoDecoder::int16Swap(int16_t s)
  {
  	unsigned char b1, b2;
  	b1 = s & 255;
  	b2 = (s >> 8) & 255;
  	return (b1 << 8) + b2;
  }

}  // namespace bebop_driver
