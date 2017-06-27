/**
Software License Agreement (BSD)

\file      ardrone3_state_callback_includes.h
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

 * ardrone3_state_callback_includes.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/ab28dab91845cd36c4d7002b55f70805deaff3c8/xml/ardrone3.xml
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#ifdef FORWARD_DECLARATIONS
namespace cb
{
  class Ardrone3MediaRecordStatePictureStateChanged;
  class Ardrone3MediaRecordStateVideoStateChanged;
  class Ardrone3MediaRecordStatePictureStateChangedV2;
  class Ardrone3MediaRecordStateVideoStateChangedV2;
  class Ardrone3MediaRecordStateVideoResolutionState;
  class Ardrone3PilotingStateFlatTrimChanged;
  class Ardrone3PilotingStateFlyingStateChanged;
  class Ardrone3PilotingStateAlertStateChanged;
  class Ardrone3PilotingStateNavigateHomeStateChanged;
  class Ardrone3PilotingStatePositionChanged;
  class Ardrone3PilotingStateSpeedChanged;
  class Ardrone3PilotingStateAttitudeChanged;
  class Ardrone3PilotingStateAutoTakeOffModeChanged;
  class Ardrone3PilotingStateAltitudeChanged;
  class Ardrone3PilotingStateGpsLocationChanged;
  class Ardrone3PilotingStateLandingStateChanged;
  class Ardrone3PilotingStateAirSpeedChanged;
  class Ardrone3PilotingStatemoveToChanged;
  class Ardrone3NetworkStateWifiScanListChanged;
  class Ardrone3NetworkStateAllWifiScanChanged;
  class Ardrone3NetworkStateWifiAuthChannelListChanged;
  class Ardrone3NetworkStateAllWifiAuthChannelChanged;
  class Ardrone3MediaStreamingStateVideoEnableChanged;
  class Ardrone3MediaStreamingStateVideoStreamModeChanged;
  class Ardrone3CameraStateOrientation;
  class Ardrone3CameraStatedefaultCameraOrientation;
  class Ardrone3CameraStateOrientationV2;
  class Ardrone3CameraStatedefaultCameraOrientationV2;
  class Ardrone3CameraStateVelocityRange;
  class Ardrone3AntiflickeringStateelectricFrequencyChanged;
  class Ardrone3AntiflickeringStatemodeChanged;
  class Ardrone3GPSStateNumberOfSatelliteChanged;
  class Ardrone3GPSStateHomeTypeAvailabilityChanged;
  class Ardrone3GPSStateHomeTypeChosenChanged;
  class Ardrone3PROStateFeatures;
  class Ardrone3AccessoryStateConnectedAccessories;
}  // namespace cb
#endif  // FORWARD_DECLARATIONS

#ifdef DEFINE_SHARED_PTRS
// Define all callback wrappers
boost::shared_ptr<cb::Ardrone3MediaRecordStatePictureStateChanged>
  ardrone3_mediarecordstate_picturestatechanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStateVideoStateChanged>
  ardrone3_mediarecordstate_videostatechanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStatePictureStateChangedV2>
  ardrone3_mediarecordstate_picturestatechangedv2_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStateVideoStateChangedV2>
  ardrone3_mediarecordstate_videostatechangedv2_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStateVideoResolutionState>
  ardrone3_mediarecordstate_videoresolutionstate_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateFlatTrimChanged>
  ardrone3_pilotingstate_flattrimchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateFlyingStateChanged>
  ardrone3_pilotingstate_flyingstatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAlertStateChanged>
  ardrone3_pilotingstate_alertstatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateNavigateHomeStateChanged>
  ardrone3_pilotingstate_navigatehomestatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStatePositionChanged>
  ardrone3_pilotingstate_positionchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateSpeedChanged>
  ardrone3_pilotingstate_speedchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAttitudeChanged>
  ardrone3_pilotingstate_attitudechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAutoTakeOffModeChanged>
  ardrone3_pilotingstate_autotakeoffmodechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAltitudeChanged>
  ardrone3_pilotingstate_altitudechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateGpsLocationChanged>
  ardrone3_pilotingstate_gpslocationchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateLandingStateChanged>
  ardrone3_pilotingstate_landingstatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAirSpeedChanged>
  ardrone3_pilotingstate_airspeedchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStatemoveToChanged>
  ardrone3_pilotingstate_movetochanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateWifiScanListChanged>
  ardrone3_networkstate_wifiscanlistchanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateAllWifiScanChanged>
  ardrone3_networkstate_allwifiscanchanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateWifiAuthChannelListChanged>
  ardrone3_networkstate_wifiauthchannellistchanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateAllWifiAuthChannelChanged>
  ardrone3_networkstate_allwifiauthchannelchanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaStreamingStateVideoEnableChanged>
  ardrone3_mediastreamingstate_videoenablechanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaStreamingStateVideoStreamModeChanged>
  ardrone3_mediastreamingstate_videostreammodechanged_ptr;
boost::shared_ptr<cb::Ardrone3CameraStateOrientation>
  ardrone3_camerastate_orientation_ptr;
boost::shared_ptr<cb::Ardrone3CameraStatedefaultCameraOrientation>
  ardrone3_camerastate_defaultcameraorientation_ptr;
boost::shared_ptr<cb::Ardrone3CameraStateOrientationV2>
  ardrone3_camerastate_orientationv2_ptr;
boost::shared_ptr<cb::Ardrone3CameraStatedefaultCameraOrientationV2>
  ardrone3_camerastate_defaultcameraorientationv2_ptr;
boost::shared_ptr<cb::Ardrone3CameraStateVelocityRange>
  ardrone3_camerastate_velocityrange_ptr;
boost::shared_ptr<cb::Ardrone3AntiflickeringStateelectricFrequencyChanged>
  ardrone3_antiflickeringstate_electricfrequencychanged_ptr;
boost::shared_ptr<cb::Ardrone3AntiflickeringStatemodeChanged>
  ardrone3_antiflickeringstate_modechanged_ptr;
boost::shared_ptr<cb::Ardrone3GPSStateNumberOfSatelliteChanged>
  ardrone3_gpsstate_numberofsatellitechanged_ptr;
boost::shared_ptr<cb::Ardrone3GPSStateHomeTypeAvailabilityChanged>
  ardrone3_gpsstate_hometypeavailabilitychanged_ptr;
boost::shared_ptr<cb::Ardrone3GPSStateHomeTypeChosenChanged>
  ardrone3_gpsstate_hometypechosenchanged_ptr;
boost::shared_ptr<cb::Ardrone3PROStateFeatures>
  ardrone3_prostate_features_ptr;
boost::shared_ptr<cb::Ardrone3AccessoryStateConnectedAccessories>
  ardrone3_accessorystate_connectedaccessories_ptr;
#endif  // DEFINE_SHARED_PTRS

#ifdef UPDTAE_CALLBACK_MAP
// Instantiate state callback wrappers
ardrone3_mediarecordstate_picturestatechanged_ptr.reset(
  new cb::Ardrone3MediaRecordStatePictureStateChanged(
    nh, priv_nh, "states/ardrone3/MediaRecordState/PictureStateChanged"));
ardrone3_mediarecordstate_videostatechanged_ptr.reset(
  new cb::Ardrone3MediaRecordStateVideoStateChanged(
    nh, priv_nh, "states/ardrone3/MediaRecordState/VideoStateChanged"));
ardrone3_mediarecordstate_picturestatechangedv2_ptr.reset(
  new cb::Ardrone3MediaRecordStatePictureStateChangedV2(
    nh, priv_nh, "states/ardrone3/MediaRecordState/PictureStateChangedV2"));
ardrone3_mediarecordstate_videostatechangedv2_ptr.reset(
  new cb::Ardrone3MediaRecordStateVideoStateChangedV2(
    nh, priv_nh, "states/ardrone3/MediaRecordState/VideoStateChangedV2"));
ardrone3_mediarecordstate_videoresolutionstate_ptr.reset(
  new cb::Ardrone3MediaRecordStateVideoResolutionState(
    nh, priv_nh, "states/ardrone3/MediaRecordState/VideoResolutionState"));
ardrone3_pilotingstate_flattrimchanged_ptr.reset(
  new cb::Ardrone3PilotingStateFlatTrimChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/FlatTrimChanged"));
ardrone3_pilotingstate_flyingstatechanged_ptr.reset(
  new cb::Ardrone3PilotingStateFlyingStateChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/FlyingStateChanged"));
ardrone3_pilotingstate_alertstatechanged_ptr.reset(
  new cb::Ardrone3PilotingStateAlertStateChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/AlertStateChanged"));
ardrone3_pilotingstate_navigatehomestatechanged_ptr.reset(
  new cb::Ardrone3PilotingStateNavigateHomeStateChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/NavigateHomeStateChanged"));
ardrone3_pilotingstate_positionchanged_ptr.reset(
  new cb::Ardrone3PilotingStatePositionChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/PositionChanged"));
ardrone3_pilotingstate_speedchanged_ptr.reset(
  new cb::Ardrone3PilotingStateSpeedChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/SpeedChanged"));
ardrone3_pilotingstate_attitudechanged_ptr.reset(
  new cb::Ardrone3PilotingStateAttitudeChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/AttitudeChanged"));
ardrone3_pilotingstate_autotakeoffmodechanged_ptr.reset(
  new cb::Ardrone3PilotingStateAutoTakeOffModeChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/AutoTakeOffModeChanged"));
ardrone3_pilotingstate_altitudechanged_ptr.reset(
  new cb::Ardrone3PilotingStateAltitudeChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/AltitudeChanged"));
ardrone3_pilotingstate_gpslocationchanged_ptr.reset(
  new cb::Ardrone3PilotingStateGpsLocationChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/GpsLocationChanged"));
ardrone3_pilotingstate_landingstatechanged_ptr.reset(
  new cb::Ardrone3PilotingStateLandingStateChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/LandingStateChanged"));
ardrone3_pilotingstate_airspeedchanged_ptr.reset(
  new cb::Ardrone3PilotingStateAirSpeedChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/AirSpeedChanged"));
ardrone3_pilotingstate_movetochanged_ptr.reset(
  new cb::Ardrone3PilotingStatemoveToChanged(
    nh, priv_nh, "states/ardrone3/PilotingState/moveToChanged"));
ardrone3_networkstate_wifiscanlistchanged_ptr.reset(
  new cb::Ardrone3NetworkStateWifiScanListChanged(
    nh, priv_nh, "states/ardrone3/NetworkState/WifiScanListChanged"));
ardrone3_networkstate_allwifiscanchanged_ptr.reset(
  new cb::Ardrone3NetworkStateAllWifiScanChanged(
    nh, priv_nh, "states/ardrone3/NetworkState/AllWifiScanChanged"));
ardrone3_networkstate_wifiauthchannellistchanged_ptr.reset(
  new cb::Ardrone3NetworkStateWifiAuthChannelListChanged(
    nh, priv_nh, "states/ardrone3/NetworkState/WifiAuthChannelListChanged"));
ardrone3_networkstate_allwifiauthchannelchanged_ptr.reset(
  new cb::Ardrone3NetworkStateAllWifiAuthChannelChanged(
    nh, priv_nh, "states/ardrone3/NetworkState/AllWifiAuthChannelChanged"));
ardrone3_mediastreamingstate_videoenablechanged_ptr.reset(
  new cb::Ardrone3MediaStreamingStateVideoEnableChanged(
    nh, priv_nh, "states/ardrone3/MediaStreamingState/VideoEnableChanged"));
ardrone3_mediastreamingstate_videostreammodechanged_ptr.reset(
  new cb::Ardrone3MediaStreamingStateVideoStreamModeChanged(
    nh, priv_nh, "states/ardrone3/MediaStreamingState/VideoStreamModeChanged"));
ardrone3_camerastate_orientation_ptr.reset(
  new cb::Ardrone3CameraStateOrientation(
    nh, priv_nh, "states/ardrone3/CameraState/Orientation"));
ardrone3_camerastate_defaultcameraorientation_ptr.reset(
  new cb::Ardrone3CameraStatedefaultCameraOrientation(
    nh, priv_nh, "states/ardrone3/CameraState/defaultCameraOrientation"));
ardrone3_camerastate_orientationv2_ptr.reset(
  new cb::Ardrone3CameraStateOrientationV2(
    nh, priv_nh, "states/ardrone3/CameraState/OrientationV2"));
ardrone3_camerastate_defaultcameraorientationv2_ptr.reset(
  new cb::Ardrone3CameraStatedefaultCameraOrientationV2(
    nh, priv_nh, "states/ardrone3/CameraState/defaultCameraOrientationV2"));
ardrone3_camerastate_velocityrange_ptr.reset(
  new cb::Ardrone3CameraStateVelocityRange(
    nh, priv_nh, "states/ardrone3/CameraState/VelocityRange"));
ardrone3_antiflickeringstate_electricfrequencychanged_ptr.reset(
  new cb::Ardrone3AntiflickeringStateelectricFrequencyChanged(
    nh, priv_nh, "states/ardrone3/AntiflickeringState/electricFrequencyChanged"));
ardrone3_antiflickeringstate_modechanged_ptr.reset(
  new cb::Ardrone3AntiflickeringStatemodeChanged(
    nh, priv_nh, "states/ardrone3/AntiflickeringState/modeChanged"));
ardrone3_gpsstate_numberofsatellitechanged_ptr.reset(
  new cb::Ardrone3GPSStateNumberOfSatelliteChanged(
    nh, priv_nh, "states/ardrone3/GPSState/NumberOfSatelliteChanged"));
ardrone3_gpsstate_hometypeavailabilitychanged_ptr.reset(
  new cb::Ardrone3GPSStateHomeTypeAvailabilityChanged(
    nh, priv_nh, "states/ardrone3/GPSState/HomeTypeAvailabilityChanged"));
ardrone3_gpsstate_hometypechosenchanged_ptr.reset(
  new cb::Ardrone3GPSStateHomeTypeChosenChanged(
    nh, priv_nh, "states/ardrone3/GPSState/HomeTypeChosenChanged"));
ardrone3_prostate_features_ptr.reset(
  new cb::Ardrone3PROStateFeatures(
    nh, priv_nh, "states/ardrone3/PROState/Features"));
ardrone3_accessorystate_connectedaccessories_ptr.reset(
  new cb::Ardrone3AccessoryStateConnectedAccessories(
    nh, priv_nh, "states/ardrone3/AccessoryState/ConnectedAccessories"));

// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediarecordstate_picturestatechanged_ptr->GetCommandKey(),
                      ardrone3_mediarecordstate_picturestatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediarecordstate_videostatechanged_ptr->GetCommandKey(),
                      ardrone3_mediarecordstate_videostatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediarecordstate_picturestatechangedv2_ptr->GetCommandKey(),
                      ardrone3_mediarecordstate_picturestatechangedv2_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediarecordstate_videostatechangedv2_ptr->GetCommandKey(),
                      ardrone3_mediarecordstate_videostatechangedv2_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediarecordstate_videoresolutionstate_ptr->GetCommandKey(),
                      ardrone3_mediarecordstate_videoresolutionstate_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_flattrimchanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_flattrimchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_flyingstatechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_flyingstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_alertstatechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_alertstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_navigatehomestatechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_navigatehomestatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_positionchanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_positionchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_speedchanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_speedchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_attitudechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_attitudechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_autotakeoffmodechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_autotakeoffmodechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_altitudechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_altitudechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_gpslocationchanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_gpslocationchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_landingstatechanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_landingstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_airspeedchanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_airspeedchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingstate_movetochanged_ptr->GetCommandKey(),
                      ardrone3_pilotingstate_movetochanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_networkstate_wifiscanlistchanged_ptr->GetCommandKey(),
                      ardrone3_networkstate_wifiscanlistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_networkstate_allwifiscanchanged_ptr->GetCommandKey(),
                      ardrone3_networkstate_allwifiscanchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_networkstate_wifiauthchannellistchanged_ptr->GetCommandKey(),
                      ardrone3_networkstate_wifiauthchannellistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_networkstate_allwifiauthchannelchanged_ptr->GetCommandKey(),
                      ardrone3_networkstate_allwifiauthchannelchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediastreamingstate_videoenablechanged_ptr->GetCommandKey(),
                      ardrone3_mediastreamingstate_videoenablechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_mediastreamingstate_videostreammodechanged_ptr->GetCommandKey(),
                      ardrone3_mediastreamingstate_videostreammodechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_camerastate_orientation_ptr->GetCommandKey(),
                      ardrone3_camerastate_orientation_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_camerastate_defaultcameraorientation_ptr->GetCommandKey(),
                      ardrone3_camerastate_defaultcameraorientation_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_camerastate_orientationv2_ptr->GetCommandKey(),
                      ardrone3_camerastate_orientationv2_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_camerastate_defaultcameraorientationv2_ptr->GetCommandKey(),
                      ardrone3_camerastate_defaultcameraorientationv2_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_camerastate_velocityrange_ptr->GetCommandKey(),
                      ardrone3_camerastate_velocityrange_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_antiflickeringstate_electricfrequencychanged_ptr->GetCommandKey(),
                      ardrone3_antiflickeringstate_electricfrequencychanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_antiflickeringstate_modechanged_ptr->GetCommandKey(),
                      ardrone3_antiflickeringstate_modechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpsstate_numberofsatellitechanged_ptr->GetCommandKey(),
                      ardrone3_gpsstate_numberofsatellitechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpsstate_hometypeavailabilitychanged_ptr->GetCommandKey(),
                      ardrone3_gpsstate_hometypeavailabilitychanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpsstate_hometypechosenchanged_ptr->GetCommandKey(),
                      ardrone3_gpsstate_hometypechosenchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_prostate_features_ptr->GetCommandKey(),
                      ardrone3_prostate_features_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_accessorystate_connectedaccessories_ptr->GetCommandKey(),
                      ardrone3_accessorystate_connectedaccessories_ptr));
#endif  // UPDTAE_CALLBACK_MAP
