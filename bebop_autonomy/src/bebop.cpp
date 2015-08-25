#include <stdexcept>
#include <boost/bind.hpp>

#include <bebop_autonomy/bebop.h>
#include <boost/thread.hpp>

#include <cmath>
#include <iostream>

namespace bebop_autonomy
{

const char* Bebop::LOG_TAG = "BEB";

void Bebop::StateChangedCallback(eARCONTROLLER_DEVICE_STATE new_state, eARCONTROLLER_ERROR error, void *bebop_void_ptr)
{
  // TODO: Log error
  Bebop* bebop_ptr_ = static_cast<Bebop*>(bebop_void_ptr);
//  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "In State Changed Callback: %d", new_state);
//  std::cout <<  "[THREAD] StateChanged: " << boost::this_thread::get_id() << std::endl;
  switch (new_state)
  {
  case ARCONTROLLER_DEVICE_STATE_STOPPED:
    ARSAL_Sem_Post(&(bebop_ptr_->state_sem_));
    break;
  case ARCONTROLLER_DEVICE_STATE_RUNNING:
    ARSAL_Sem_Post(&(bebop_ptr_->state_sem_));
    break;
  }
}

void Bebop::CommandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY cmd_key, ARCONTROLLER_DICTIONARY_ELEMENT_t *element_dict_ptr, void *bebop_void_ptr)
{
  Bebop* bebop_ptr_ = static_cast<Bebop*>(bebop_void_ptr);
  const ARCONTROLLER_Device_t* dev_ctr_ptr = bebop_ptr_->GetControllerCstPtr();
//  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "In Command Received Callback: %d", cmd_key);
//  std::cout <<  "[THREAD] CommandRecv: " << boost::this_thread::get_id() << std::endl;

  ARCONTROLLER_DICTIONARY_ARG_t *arg_ptr = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *single_element_ptr = NULL;

  if (element_dict_ptr)
  {
    // We are only interested in single key dictionaries
    HASH_FIND_STR (element_dict_ptr, ARCONTROLLER_DICTIONARY_SINGLE_KEY, single_element_ptr);

    if (single_element_ptr)
    {
      if (single_element_ptr->arguments)
        ;//std::cout << "Key: " << single_element_ptr->arguments->argument << " Value: " << single_element_ptr->arguments->value.Float << std::endl;
        //ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Single element key: %s value: %s", single_element_ptr->arguments->argument);
    }
//    if (element_dict_ptr && element_dict_ptr->arguments)
//      ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Command: %d Key: %s Number of items: %u Argument Key: %s",
//                  cmd_key, element_dict_ptr->key, element_dict_ptr->arguments->hh.tbl->num_items, element_dict_ptr->arguments->argument);
  }

  if (dev_ctr_ptr)
  {
    //;
  }
}

void Bebop::FrameReceivedCallback(ARCONTROLLER_Frame_t *frame, void *bebop_void_ptr_)
{
  if (!frame)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Received frame is NULL");
  }
  Bebop* bebop_ptr_ = static_cast<Bebop*>(bebop_void_ptr_);
//  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "In Frame Received Callback: %u %u", frame->width, frame->used);
//  std::cout <<  "[THREAD] FrameRecv: " << boost::this_thread::get_id() << std::endl;
  // TODO: FixMe
  frame->width = 640;
  frame->height = 368;
  if (!bebop_ptr_->video_decoder_.Decode(frame))
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Video decode failed");
  }
}


Bebop::Bebop():
  connected_(false),
  device_ptr_(NULL),
  device_controller_ptr_(NULL),
  error_(ARCONTROLLER_OK),
  device_state_(ARCONTROLLER_DEVICE_STATE_MAX),
  video_decoder_()
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Bebop Cnstr()");
}

Bebop::~Bebop()
{
  // This is the last resort, the program must run Cleanup() fo
  // proper disconnection and free
  if (device_ptr_) ARDISCOVERY_Device_Delete(&device_ptr_);
  if (device_controller_ptr_) ARCONTROLLER_Device_Delete(&device_controller_ptr_);
}

void Bebop::Connect()
{
  try
  {
    if (connected_) throw std::runtime_error("Already inited");

    // TODO: Error checking;
    ARSAL_Sem_Init(&state_sem_, 0, 0);

    eARDISCOVERY_ERROR error_discovery = ARDISCOVERY_OK;
    device_ptr_ = ARDISCOVERY_Device_New(&error_discovery);

    if (error_discovery != ARDISCOVERY_OK)
    {
      throw std::runtime_error("Discovery failed: " + std::string(ARDISCOVERY_Error_ToString(error_discovery)));
    }

    error_discovery = ARDISCOVERY_Device_InitWifi(device_ptr_,
                                                  ARDISCOVERY_PRODUCT_ARDRONE, "Bebop",
                                                  "192.168.42.1", 44444);

    if (error_discovery != ARDISCOVERY_OK)
    {
      throw std::runtime_error("Discovery failed: " + std::string(ARDISCOVERY_Error_ToString(error_discovery)));
    }

    device_controller_ptr_ = ARCONTROLLER_Device_New(device_ptr_, &error_);
    ThrowOnCtrlError(error_, "Creation of device controller failed: ");

    ARDISCOVERY_Device_Delete(&device_ptr_);

    ThrowOnCtrlError(
          ARCONTROLLER_Device_AddStateChangedCallback(device_controller_ptr_, Bebop::StateChangedCallback, (void*) this),
          "Registering state callback failed");
    ThrowOnCtrlError(
          ARCONTROLLER_Device_AddCommandReceivedCallback(device_controller_ptr_, Bebop::CommandReceivedCallback, (void*) this),
          "Registering command callback failed");
    // third argument is frame timeout callback
    ThrowOnCtrlError(
          ARCONTROLLER_Device_SetVideoReceiveCallback (device_controller_ptr_, Bebop::FrameReceivedCallback, NULL , (void*) this),
          "Registering video callback failed");


    ThrowOnCtrlError(ARCONTROLLER_Device_Start(device_controller_ptr_), "Controller device start failed");

    // Semaphore is touched inside the StateCallback
    ARSAL_Sem_Wait(&state_sem_);

    device_state_ = ARCONTROLLER_Device_GetState(device_controller_ptr_, &error_);
    if ((error_ != ARCONTROLLER_OK) || (device_state_ != ARCONTROLLER_DEVICE_STATE_RUNNING))
    {
      throw std::runtime_error("Waiting for device failed: " + std::string(ARCONTROLLER_Error_ToString(error_)));
    }

    // Start video streaming
    ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendMediaStreamingVideoEnable(
                       device_controller_ptr_->aRDrone3, 1), "Starting video stream failed.");

  }
  catch (const std::exception& e)
  {
    Cleanup();
    throw e;
  }

  std::cout <<  "[THREAD] Init: " << boost::this_thread::get_id() << std::endl;
  connected_ = true;
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Inited() and Dis");
}

void Bebop::Cleanup()
{
  if (device_controller_ptr_)
  {
    device_state_ = ARCONTROLLER_Device_GetState(device_controller_ptr_, &error_);
    if ((error_ == ARCONTROLLER_OK) && (device_state_ != ARCONTROLLER_DEVICE_STATE_STOPPED))
    {
      // Say disconnecting
      error_ = ARCONTROLLER_Device_Stop(device_controller_ptr_);
      if (error_ == ARCONTROLLER_OK)
      {
        ARSAL_Sem_Wait(&state_sem_);
      }
    }
    ARCONTROLLER_Device_Delete(&device_controller_ptr_);
  }
  ARSAL_Sem_Destroy(&state_sem_);
}

bool Bebop::Disconnect()
{
  if (!connected_) return false;
  Cleanup();
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "-- END --");
  return true;
}

void Bebop::Takeoff()
{
  ThrowOnInternalError("Takeoff failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingTakeOff(device_controller_ptr_->aRDrone3),
        "Takeoff failed");
}

void Bebop::Land()
{
  ThrowOnInternalError("Land failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingLanding(device_controller_ptr_->aRDrone3),
        "Land failed");
}

void Bebop::Move(const double &roll, const double &pitch, const double &yaw_speed, const double &gaz_speed)
{
  // TODO: Bound check
  ThrowOnInternalError("Move failure");
//  ThrowOnCtrlError(
//        device_controller_ptr_->aRDrone3->sendPilotingPCMD(
//          device_controller_ptr_->aRDrone3,
//          1,
//          roll * 100.0,
//          pitch * 100.0,
//          yaw_speed * 100.0,
//          gaz_speed * 100.0,
//          0));
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->setPilotingPCMD(
          device_controller_ptr_->aRDrone3,
          1,
          roll * 100.0,
          pitch * 100.0,
          yaw_speed * 100.0,
          gaz_speed * 100.0,
          0));
}

// in degrees?
void Bebop::MoveCamera(const double &tilt, const double &pan)
{
  ThrowOnInternalError("Camera Move Failure");
  ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendCameraOrientation(
                     device_controller_ptr_->aRDrone3,
                     static_cast<int8_t>(tilt),
                     static_cast<int8_t>(pan)));
}

void Bebop::ThrowOnInternalError(const std::string &message)
{
  if (!connected_ || !device_controller_ptr_)
  {
    throw std::runtime_error(message);
  }
}

void Bebop::ThrowOnCtrlError(const eARCONTROLLER_ERROR &error, const std::string &message)
{
  if (error != ARCONTROLLER_OK)
  {
    throw std::runtime_error(message + std::string(ARCONTROLLER_Error_ToString(error)));
  }
}

}  // namespace bebop_autonomys
