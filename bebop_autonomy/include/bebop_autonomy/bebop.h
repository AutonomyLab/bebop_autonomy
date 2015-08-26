#ifndef BEBOP_H
#define BEBOP_H

#define BEBOP_ERR_STR_SZ  100

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>

extern "C"
{
  #include "libARSAL/ARSAL.h"
  #include "libARController/ARController.h"
  #include "libARDiscovery/ARDiscovery.h"
}

#include "bebop_autonomy/bebop_video_decoder.h"


// Debug
#include <iostream>
#include <fstream>

namespace bebop_autonomy
{

namespace util
{

template<typename T>
class AtomicVar
{
private:
  boost::atomic<T> v_;

public:
  AtomicVar();
  explicit AtomicVar(const T& v): v_(v) {}
  inline T Get() const {return v_;}
  void Set(const T& v)
  {
    v_ = v;
  }
};

}
class Bebop
{
private:
  static const char* LOG_TAG;
  bool connected_;
  ARDISCOVERY_Device_t* device_ptr_;
  ARCONTROLLER_Device_t* device_controller_ptr_;
  eARCONTROLLER_ERROR error_;
  eARCONTROLLER_DEVICE_STATE device_state_;
  ARSAL_Sem_t state_sem_;
  VideoDecoder video_decoder_;

  // sync
  util::AtomicVar<bool> frame_avail_flag_;

  static void StateChangedCallback(eARCONTROLLER_DEVICE_STATE new_state, eARCONTROLLER_ERROR error, void *bebop_void_ptr);
  static void CommandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY cmd_key, ARCONTROLLER_DICTIONARY_ELEMENT_t* element_dict_ptr, void* bebop_void_ptr);
  static void FrameReceivedCallback(ARCONTROLLER_Frame_t *frame, void *bebop_void_ptr_);

  void Cleanup();

  void ThrowOnInternalError(const std::string& message = std::string());
  void ThrowOnCtrlError(const eARCONTROLLER_ERROR& error, const std::string& message = std::string());

public:

  inline ARSAL_Sem_t* GetStateSemPtr() {return &state_sem_;}
  inline const ARCONTROLLER_Device_t* GetControllerCstPtr() const {return device_controller_ptr_;}

  // Make this atomic
  inline bool IsConnected() const {return connected_;}

  Bebop(ARSAL_Print_Callback_t custom_print_callback = 0);
  ~Bebop();

  inline util::AtomicVar<bool>& FrameAvailableFlag() {return frame_avail_flag_;}

  void Connect();
  bool Disconnect();

  void Takeoff();
  void Land();
  inline const VideoDecoder& Decoder() const {return video_decoder_;}

  // -1..1
  void Move(const double& roll, const double& pitch, const double& yaw_speed, const double& gaz_speed);
  void MoveCamera(const double& tilt, const double& pan);

  // Debug
//  std::ofstream out_file;
};

}  // namespace bebop_autonomy


#endif  // BEBOP_H
