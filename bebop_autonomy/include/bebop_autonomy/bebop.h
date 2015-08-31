#ifndef BEBOP_H
#define BEBOP_H

#define BEBOP_ERR_STR_SZ  150

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

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
#include <sys/syscall.h>
#include <sys/types.h>
#include <map>


#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace bebop_autonomy
{

namespace util
{
inline long int GetLWPId()
{
  return syscall(SYS_gettid);
}

}  // namespace util

class CommandBase
{
protected:
  eARCONTROLLER_DICTIONARY_KEY cmd_key_;
  ARCONTROLLER_DICTIONARY_ARG_t* arg;
  mutable boost::mutex mutex_;
  ros::Publisher ros_pub_;

public:
  explicit CommandBase(eARCONTROLLER_DICTIONARY_KEY cmd_key)
    : cmd_key_(cmd_key), arg(NULL)
  {
    ;
  }

  virtual ~CommandBase()
  {}

  virtual void Update(const ARCONTROLLER_DICTIONARY_ARG_t* arg) = 0;
};

class CameraState : public CommandBase
{
public:
  boost::atomic<float> tilt;
  boost::atomic<float> pan;

  CameraState(ros::NodeHandle& nh, const std::string& topic)
    : CommandBase(ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION)
  {
    ros_pub_ = nh.advertise<std_msgs::Float32>(topic, 10);
  }

  void Update(const ARCONTROLLER_DICTIONARY_ARG_t* arg)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "Camera State Argument: %s", arg->argument);
    if (strcmp(arg->argument, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION_TILT) == 0)
    {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "Tilt: %f", arg->value.Float);
      tilt = arg->value.Float;
    }
    else if (strcmp(arg->argument, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION_PAN) == 0)
    {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "Pan: %f", arg->value.Float);
      pan = arg->value.Float;
    }
  }
};

class AttitudeChanged : public CommandBase
{
public:
  AttitudeChanged(ros::NodeHandle& nh, const std::string& topic)
    : CommandBase(ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED)
  {
    ros_pub_ = nh.advertise<std_msgs::Float32>(topic, 10);
  }


  std_msgs::Float32ConstPtr GetDataCstPtr() const
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    return msg_ptr;
  }


private:
  std_msgs::Float32Ptr msg_ptr;

  void Update(const ARCONTROLLER_DICTIONARY_ARG_t *arguments)
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    arg = NULL;
    HASH_FIND_STR (arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL, arg);

    msg_ptr.reset(new std_msgs::Float32);
    if (arg)
    {
      msg_ptr->data = arg->value.Float;
    }

    ros_pub_.publish(msg_ptr);
//    HASH_FIND_STR (arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH, arg);
//    if (arg)
//    {
//      ARSAL_PRINT(ARSAL_PRINT_ERROR, "LOG_TAG", "Pitch: %f", arg->value.Float);
//    }
//    ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "Attitude Argument: %s", arg->argument);
//    if (strcmp(arg->argument, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL) == 0)
//    {
//      ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "roll: %f", arg->value.Float);
//      roll = arg->value.Float;
//    }
//    else if (strcmp(arg->argument, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH) == 0)
//    {
//      ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "Pitch: %f", arg->value.Float);
//      pitch = arg->value.Float;
//    }
//    else if (strcmp(arg->argument, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_YAW) == 0)
//    {
//      ARSAL_PRINT(ARSAL_PRINT_ERROR, "Mani", "Yaw: %f", arg->value.Float);
//      yaw = arg->value.Float;
//    }
  }
};

class Bebop
{
private:
  static const char* LOG_TAG;
  boost::atomic<bool> is_connected_;
  ARDISCOVERY_Device_t* device_ptr_;
  ARCONTROLLER_Device_t* device_controller_ptr_;
  eARCONTROLLER_ERROR error_;
  eARCONTROLLER_DEVICE_STATE device_state_;
  ARSAL_Sem_t state_sem_;
  VideoDecoder video_decoder_;

  boost::shared_ptr<AttitudeChanged> attitude_changed_ptr_;
  std::map<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<CommandBase> > command_map_;

  // sync
  mutable boost::condition_variable frame_avail_cond_;
  mutable boost::mutex frame_avail_mutex_;
  mutable bool is_frame_avail_;

  static void BatteryStateChangedCallback(uint8_t percent, void *bebop_void_ptr);
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
  inline bool IsConnected() const {return is_connected_;}

  Bebop(ARSAL_Print_Callback_t custom_print_callback = 0);
  ~Bebop();

  void Connect(ros::NodeHandle& nh);
  bool Disconnect();

  void Takeoff();
  void Land();

  // -1..1
  void Move(const double& roll, const double& pitch, const double& gaz_speed, const double& yaw_speed);
  void MoveCamera(const double& tilt, const double& pan);

  // This function is blocking and runs in the caller's thread's context
  // which is different from FrameReceivedCallback's context
  bool GetFrontCameraFrame(std::vector<uint8_t>& buffer, uint32_t &width, uint32_t &height) const;
  uint32_t GetFrontCameraFrameWidth() const {return video_decoder_.GetFrameWidth();}
  uint32_t GetFrontCameraFrameHeight() const {return video_decoder_.GetFrameHeight();}

  // Debug
//  std::ofstream out_file;
};

}  // namespace bebop_autonomy


#endif  // BEBOP_H
