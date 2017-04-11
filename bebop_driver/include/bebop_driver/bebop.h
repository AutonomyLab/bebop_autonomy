/**
Software License Agreement (BSD)

\file      bebop.h
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
#ifndef BEBOP_AUTONOMY_BEBOP_H
#define BEBOP_AUTONOMY_BEBOP_H

#define BEBOP_ERR_STR_SZ  150

#include <sys/syscall.h>
#include <sys/types.h>

#include <string>
#include <vector>
#include <utility>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

extern "C"
{
  #include "libARSAL/ARSAL.h"
  #include "libARController/ARController.h"
}

// Forward declarations
extern "C"
{
  struct ARDISCOVERY_Device_t;
}
namespace ros
{
  class NodeHandle;
}

namespace bebop_driver
{

namespace util
{
inline long int GetLWPId()
{
  return (syscall(SYS_gettid));
}

}  // namespace util

// Forward declarations
class BebopArdrone3Config;
class VideoDecoder;
namespace cb
{
  class AbstractCommand;
}  // namespace cb
#define FORWARD_DECLARATIONS
#include "bebop_driver/autogenerated/common_state_callback_includes.h"
#include "bebop_driver/autogenerated/ardrone3_state_callback_includes.h"
#include "bebop_driver/autogenerated/ardrone3_setting_callback_includes.h"
#undef FORWARD_DECLARATIONS

class Bebop
{
private:
  static const char* LOG_TAG;
  boost::atomic<bool> is_connected_;
  boost::atomic<bool> is_streaming_started_;
  ARDISCOVERY_Device_t* device_ptr_;
  ARCONTROLLER_Device_t* device_controller_ptr_;
  eARCONTROLLER_ERROR error_;
  eARCONTROLLER_DEVICE_STATE device_state_;
  ARSAL_Sem_t state_sem_;
  boost::shared_ptr<VideoDecoder> video_decoder_ptr_;
  std::string bebop_ip_;

//  boost::mutex callback_map_mutex_;
  typedef std::map<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> > callback_map_t;
  typedef std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> > callback_map_pair_t;
  callback_map_t callback_map_;

  // sync
  mutable boost::condition_variable frame_avail_cond_;
  mutable boost::mutex frame_avail_mutex_;
  mutable bool is_frame_avail_;

  static void StateChangedCallback(
      eARCONTROLLER_DEVICE_STATE new_state,
      eARCONTROLLER_ERROR error,
      void *bebop_void_ptr);

  static void CommandReceivedCallback(
      eARCONTROLLER_DICTIONARY_KEY cmd_key,
      ARCONTROLLER_DICTIONARY_ELEMENT_t* element_dict_ptr,
      void* bebop_void_ptr);

  static eARCONTROLLER_ERROR FrameReceivedCallback(ARCONTROLLER_Frame_t *frame,
      void *bebop_void_ptr);

  static eARCONTROLLER_ERROR DecoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec,
      void *bebop_void_ptr);

  // nothrow
  void Cleanup();

  void ThrowOnInternalError(const std::string& message = std::string());
  void ThrowOnCtrlError(const eARCONTROLLER_ERROR& error, const std::string& message = std::string());

public:
  // Navdata Callbacks
#define DEFINE_SHARED_PTRS
#include "bebop_driver/autogenerated/common_state_callback_includes.h"
#include "bebop_driver/autogenerated/ardrone3_state_callback_includes.h"
#include "bebop_driver/autogenerated/ardrone3_setting_callback_includes.h"
#undef DEFINE_SHARED_PTRS

  inline ARSAL_Sem_t* GetStateSemPtr() {return &state_sem_;}
  inline const ARCONTROLLER_Device_t* GetControllerCstPtr() const {return device_controller_ptr_;}

  inline bool IsConnected() const {return is_connected_;}
  inline bool IsStreamingStarted() const {return is_streaming_started_;}

  explicit Bebop(ARSAL_Print_Callback_t custom_print_callback = 0);
  ~Bebop();

  void Connect(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& bebop_ip = "192.168.42.1");
  void StartStreaming();

  // Disable all data callback and streaming (nothrow)
  void StopStreaming();
  // nothrow
  void Disconnect();

  void SetDate(const std::string &date);
  void RequestAllSettings();
  void ResetAllSettings();
  void UpdateSettings(const bebop_driver::BebopArdrone3Config& config);

  void Takeoff();
  void Land();
  void Emergency();
  void FlatTrim();
  // false: Stop, true: Start
  void NavigateHome(const bool& start_stop);
  void StartAutonomousFlight(const std::string &filepath);
  void PauseAutonomousFlight();
  void StopAutonomousFlight();
  void AnimationFlip(const uint8_t& anim_id);

  // -1..1
  void Move(const double& roll, const double& pitch, const double& gaz_speed, const double& yaw_speed);
  void MoveBy(const float& dX, const float& dY, const float& dZ, const float& dPsi);
  void MoveCamera(const double& tilt, const double& pan);

  void TakeSnapshot();

  /**
   * @brief Set the format of the taken pictures
   * @param format 0: Raw image, 1: 4:3 jpeg photo, 2: 16:9 snapshot from camera, 3: jpeg fisheye image only
   */
  void SetPictureFormat(const int& format);
  // exposure should be between -3.0 and +3.0
  void SetExposure(const float& exposure);
  // true: start, false: stop
  void ToggleVideoRecording(const bool start);

  // This function is blocking and runs in the caller's thread's context
  // which is different from FrameReceivedCallback's context
  bool GetFrontCameraFrame(std::vector<uint8_t>& buffer, uint32_t &width, uint32_t &height) const;
  uint32_t GetFrontCameraFrameWidth() const;
  uint32_t GetFrontCameraFrameHeight() const;
};

}  // namespace bebop_driver


#endif  // BEBOP_AUTONOMY_BEBOP_H
