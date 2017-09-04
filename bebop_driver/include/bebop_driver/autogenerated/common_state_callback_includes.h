/**
Software License Agreement (BSD)

\file      common_state_callback_includes.h
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

 * common_state_callback_includes.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/ab28dab91845cd36c4d7002b55f70805deaff3c8/xml/common.xml
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#ifdef FORWARD_DECLARATIONS
namespace cb
{
  class CommonCommonStateAllStatesChanged;
  class CommonCommonStateBatteryStateChanged;
  class CommonCommonStateMassStorageStateListChanged;
  class CommonCommonStateMassStorageInfoStateListChanged;
  class CommonCommonStateCurrentDateChanged;
  class CommonCommonStateCurrentTimeChanged;
  class CommonCommonStateMassStorageInfoRemainingListChanged;
  class CommonCommonStateWifiSignalChanged;
  class CommonCommonStateSensorsStatesListChanged;
  class CommonCommonStateProductModel;
  class CommonCommonStateCountryListKnown;
  class CommonCommonStateDeprecatedMassStorageContentChanged;
  class CommonCommonStateMassStorageContent;
  class CommonCommonStateMassStorageContentForCurrentRun;
  class CommonCommonStateVideoRecordingTimestamp;
  class CommonOverHeatStateOverHeatChanged;
  class CommonOverHeatStateOverHeatRegulationChanged;
  class CommonMavlinkStateMavlinkFilePlayingStateChanged;
  class CommonMavlinkStateMavlinkPlayErrorStateChanged;
  class CommonMavlinkStateMissionItemExecuted;
  class CommonCalibrationStateMagnetoCalibrationStateChanged;
  class CommonCalibrationStateMagnetoCalibrationRequiredState;
  class CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged;
  class CommonCalibrationStateMagnetoCalibrationStartedChanged;
  class CommonCalibrationStatePitotCalibrationStateChanged;
  class CommonFlightPlanStateAvailabilityStateChanged;
  class CommonFlightPlanStateComponentStateListChanged;
  class CommonFlightPlanStateLockStateChanged;
  class CommonARLibsVersionsStateControllerLibARCommandsVersion;
  class CommonARLibsVersionsStateSkyControllerLibARCommandsVersion;
  class CommonARLibsVersionsStateDeviceLibARCommandsVersion;
  class CommonAudioStateAudioStreamingRunning;
  class CommonHeadlightsStateintensityChanged;
  class CommonAnimationsStateList;
  class CommonAccessoryStateSupportedAccessoriesListChanged;
  class CommonAccessoryStateAccessoryConfigChanged;
  class CommonAccessoryStateAccessoryConfigModificationEnabled;
  class CommonChargerStateMaxChargeRateChanged;
  class CommonChargerStateCurrentChargeStateChanged;
  class CommonChargerStateLastChargeRateChanged;
  class CommonChargerStateChargingInfo;
  class CommonRunStateRunIdChanged;
}  // namespace cb
#endif  // FORWARD_DECLARATIONS

#ifdef DEFINE_SHARED_PTRS
// Define all callback wrappers
boost::shared_ptr<cb::CommonCommonStateAllStatesChanged>
  common_commonstate_allstateschanged_ptr;
boost::shared_ptr<cb::CommonCommonStateBatteryStateChanged>
  common_commonstate_batterystatechanged_ptr;
boost::shared_ptr<cb::CommonCommonStateMassStorageStateListChanged>
  common_commonstate_massstoragestatelistchanged_ptr;
boost::shared_ptr<cb::CommonCommonStateMassStorageInfoStateListChanged>
  common_commonstate_massstorageinfostatelistchanged_ptr;
boost::shared_ptr<cb::CommonCommonStateCurrentDateChanged>
  common_commonstate_currentdatechanged_ptr;
boost::shared_ptr<cb::CommonCommonStateCurrentTimeChanged>
  common_commonstate_currenttimechanged_ptr;
boost::shared_ptr<cb::CommonCommonStateMassStorageInfoRemainingListChanged>
  common_commonstate_massstorageinforemaininglistchanged_ptr;
boost::shared_ptr<cb::CommonCommonStateWifiSignalChanged>
  common_commonstate_wifisignalchanged_ptr;
boost::shared_ptr<cb::CommonCommonStateSensorsStatesListChanged>
  common_commonstate_sensorsstateslistchanged_ptr;
boost::shared_ptr<cb::CommonCommonStateProductModel>
  common_commonstate_productmodel_ptr;
boost::shared_ptr<cb::CommonCommonStateCountryListKnown>
  common_commonstate_countrylistknown_ptr;
boost::shared_ptr<cb::CommonCommonStateDeprecatedMassStorageContentChanged>
  common_commonstate_deprecatedmassstoragecontentchanged_ptr;
boost::shared_ptr<cb::CommonCommonStateMassStorageContent>
  common_commonstate_massstoragecontent_ptr;
boost::shared_ptr<cb::CommonCommonStateMassStorageContentForCurrentRun>
  common_commonstate_massstoragecontentforcurrentrun_ptr;
boost::shared_ptr<cb::CommonCommonStateVideoRecordingTimestamp>
  common_commonstate_videorecordingtimestamp_ptr;
boost::shared_ptr<cb::CommonOverHeatStateOverHeatChanged>
  common_overheatstate_overheatchanged_ptr;
boost::shared_ptr<cb::CommonOverHeatStateOverHeatRegulationChanged>
  common_overheatstate_overheatregulationchanged_ptr;
boost::shared_ptr<cb::CommonMavlinkStateMavlinkFilePlayingStateChanged>
  common_mavlinkstate_mavlinkfileplayingstatechanged_ptr;
boost::shared_ptr<cb::CommonMavlinkStateMavlinkPlayErrorStateChanged>
  common_mavlinkstate_mavlinkplayerrorstatechanged_ptr;
boost::shared_ptr<cb::CommonMavlinkStateMissionItemExecuted>
  common_mavlinkstate_missionitemexecuted_ptr;
boost::shared_ptr<cb::CommonCalibrationStateMagnetoCalibrationStateChanged>
  common_calibrationstate_magnetocalibrationstatechanged_ptr;
boost::shared_ptr<cb::CommonCalibrationStateMagnetoCalibrationRequiredState>
  common_calibrationstate_magnetocalibrationrequiredstate_ptr;
boost::shared_ptr<cb::CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged>
  common_calibrationstate_magnetocalibrationaxistocalibratechanged_ptr;
boost::shared_ptr<cb::CommonCalibrationStateMagnetoCalibrationStartedChanged>
  common_calibrationstate_magnetocalibrationstartedchanged_ptr;
boost::shared_ptr<cb::CommonCalibrationStatePitotCalibrationStateChanged>
  common_calibrationstate_pitotcalibrationstatechanged_ptr;
boost::shared_ptr<cb::CommonFlightPlanStateAvailabilityStateChanged>
  common_flightplanstate_availabilitystatechanged_ptr;
boost::shared_ptr<cb::CommonFlightPlanStateComponentStateListChanged>
  common_flightplanstate_componentstatelistchanged_ptr;
boost::shared_ptr<cb::CommonFlightPlanStateLockStateChanged>
  common_flightplanstate_lockstatechanged_ptr;
boost::shared_ptr<cb::CommonARLibsVersionsStateControllerLibARCommandsVersion>
  common_arlibsversionsstate_controllerlibarcommandsversion_ptr;
boost::shared_ptr<cb::CommonARLibsVersionsStateSkyControllerLibARCommandsVersion>
  common_arlibsversionsstate_skycontrollerlibarcommandsversion_ptr;
boost::shared_ptr<cb::CommonARLibsVersionsStateDeviceLibARCommandsVersion>
  common_arlibsversionsstate_devicelibarcommandsversion_ptr;
boost::shared_ptr<cb::CommonAudioStateAudioStreamingRunning>
  common_audiostate_audiostreamingrunning_ptr;
boost::shared_ptr<cb::CommonHeadlightsStateintensityChanged>
  common_headlightsstate_intensitychanged_ptr;
boost::shared_ptr<cb::CommonAnimationsStateList>
  common_animationsstate_list_ptr;
boost::shared_ptr<cb::CommonAccessoryStateSupportedAccessoriesListChanged>
  common_accessorystate_supportedaccessorieslistchanged_ptr;
boost::shared_ptr<cb::CommonAccessoryStateAccessoryConfigChanged>
  common_accessorystate_accessoryconfigchanged_ptr;
boost::shared_ptr<cb::CommonAccessoryStateAccessoryConfigModificationEnabled>
  common_accessorystate_accessoryconfigmodificationenabled_ptr;
boost::shared_ptr<cb::CommonChargerStateMaxChargeRateChanged>
  common_chargerstate_maxchargeratechanged_ptr;
boost::shared_ptr<cb::CommonChargerStateCurrentChargeStateChanged>
  common_chargerstate_currentchargestatechanged_ptr;
boost::shared_ptr<cb::CommonChargerStateLastChargeRateChanged>
  common_chargerstate_lastchargeratechanged_ptr;
boost::shared_ptr<cb::CommonChargerStateChargingInfo>
  common_chargerstate_charginginfo_ptr;
boost::shared_ptr<cb::CommonRunStateRunIdChanged>
  common_runstate_runidchanged_ptr;
#endif  // DEFINE_SHARED_PTRS

#ifdef UPDTAE_CALLBACK_MAP
// Instantiate state callback wrappers
common_commonstate_allstateschanged_ptr.reset(
  new cb::CommonCommonStateAllStatesChanged(
    nh, priv_nh, "states/common/CommonState/AllStatesChanged"));
common_commonstate_batterystatechanged_ptr.reset(
  new cb::CommonCommonStateBatteryStateChanged(
    nh, priv_nh, "states/common/CommonState/BatteryStateChanged"));
common_commonstate_massstoragestatelistchanged_ptr.reset(
  new cb::CommonCommonStateMassStorageStateListChanged(
    nh, priv_nh, "states/common/CommonState/MassStorageStateListChanged"));
common_commonstate_massstorageinfostatelistchanged_ptr.reset(
  new cb::CommonCommonStateMassStorageInfoStateListChanged(
    nh, priv_nh, "states/common/CommonState/MassStorageInfoStateListChanged"));
common_commonstate_currentdatechanged_ptr.reset(
  new cb::CommonCommonStateCurrentDateChanged(
    nh, priv_nh, "states/common/CommonState/CurrentDateChanged"));
common_commonstate_currenttimechanged_ptr.reset(
  new cb::CommonCommonStateCurrentTimeChanged(
    nh, priv_nh, "states/common/CommonState/CurrentTimeChanged"));
common_commonstate_massstorageinforemaininglistchanged_ptr.reset(
  new cb::CommonCommonStateMassStorageInfoRemainingListChanged(
    nh, priv_nh, "states/common/CommonState/MassStorageInfoRemainingListChanged"));
common_commonstate_wifisignalchanged_ptr.reset(
  new cb::CommonCommonStateWifiSignalChanged(
    nh, priv_nh, "states/common/CommonState/WifiSignalChanged"));
common_commonstate_sensorsstateslistchanged_ptr.reset(
  new cb::CommonCommonStateSensorsStatesListChanged(
    nh, priv_nh, "states/common/CommonState/SensorsStatesListChanged"));
common_commonstate_productmodel_ptr.reset(
  new cb::CommonCommonStateProductModel(
    nh, priv_nh, "states/common/CommonState/ProductModel"));
common_commonstate_countrylistknown_ptr.reset(
  new cb::CommonCommonStateCountryListKnown(
    nh, priv_nh, "states/common/CommonState/CountryListKnown"));
common_commonstate_deprecatedmassstoragecontentchanged_ptr.reset(
  new cb::CommonCommonStateDeprecatedMassStorageContentChanged(
    nh, priv_nh, "states/common/CommonState/DeprecatedMassStorageContentChanged"));
common_commonstate_massstoragecontent_ptr.reset(
  new cb::CommonCommonStateMassStorageContent(
    nh, priv_nh, "states/common/CommonState/MassStorageContent"));
common_commonstate_massstoragecontentforcurrentrun_ptr.reset(
  new cb::CommonCommonStateMassStorageContentForCurrentRun(
    nh, priv_nh, "states/common/CommonState/MassStorageContentForCurrentRun"));
common_commonstate_videorecordingtimestamp_ptr.reset(
  new cb::CommonCommonStateVideoRecordingTimestamp(
    nh, priv_nh, "states/common/CommonState/VideoRecordingTimestamp"));
common_overheatstate_overheatchanged_ptr.reset(
  new cb::CommonOverHeatStateOverHeatChanged(
    nh, priv_nh, "states/common/OverHeatState/OverHeatChanged"));
common_overheatstate_overheatregulationchanged_ptr.reset(
  new cb::CommonOverHeatStateOverHeatRegulationChanged(
    nh, priv_nh, "states/common/OverHeatState/OverHeatRegulationChanged"));
common_mavlinkstate_mavlinkfileplayingstatechanged_ptr.reset(
  new cb::CommonMavlinkStateMavlinkFilePlayingStateChanged(
    nh, priv_nh, "states/common/MavlinkState/MavlinkFilePlayingStateChanged"));
common_mavlinkstate_mavlinkplayerrorstatechanged_ptr.reset(
  new cb::CommonMavlinkStateMavlinkPlayErrorStateChanged(
    nh, priv_nh, "states/common/MavlinkState/MavlinkPlayErrorStateChanged"));
common_mavlinkstate_missionitemexecuted_ptr.reset(
  new cb::CommonMavlinkStateMissionItemExecuted(
    nh, priv_nh, "states/common/MavlinkState/MissionItemExecuted"));
common_calibrationstate_magnetocalibrationstatechanged_ptr.reset(
  new cb::CommonCalibrationStateMagnetoCalibrationStateChanged(
    nh, priv_nh, "states/common/CalibrationState/MagnetoCalibrationStateChanged"));
common_calibrationstate_magnetocalibrationrequiredstate_ptr.reset(
  new cb::CommonCalibrationStateMagnetoCalibrationRequiredState(
    nh, priv_nh, "states/common/CalibrationState/MagnetoCalibrationRequiredState"));
common_calibrationstate_magnetocalibrationaxistocalibratechanged_ptr.reset(
  new cb::CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged(
    nh, priv_nh, "states/common/CalibrationState/MagnetoCalibrationAxisToCalibrateChanged"));
common_calibrationstate_magnetocalibrationstartedchanged_ptr.reset(
  new cb::CommonCalibrationStateMagnetoCalibrationStartedChanged(
    nh, priv_nh, "states/common/CalibrationState/MagnetoCalibrationStartedChanged"));
common_calibrationstate_pitotcalibrationstatechanged_ptr.reset(
  new cb::CommonCalibrationStatePitotCalibrationStateChanged(
    nh, priv_nh, "states/common/CalibrationState/PitotCalibrationStateChanged"));
common_flightplanstate_availabilitystatechanged_ptr.reset(
  new cb::CommonFlightPlanStateAvailabilityStateChanged(
    nh, priv_nh, "states/common/FlightPlanState/AvailabilityStateChanged"));
common_flightplanstate_componentstatelistchanged_ptr.reset(
  new cb::CommonFlightPlanStateComponentStateListChanged(
    nh, priv_nh, "states/common/FlightPlanState/ComponentStateListChanged"));
common_flightplanstate_lockstatechanged_ptr.reset(
  new cb::CommonFlightPlanStateLockStateChanged(
    nh, priv_nh, "states/common/FlightPlanState/LockStateChanged"));
common_arlibsversionsstate_controllerlibarcommandsversion_ptr.reset(
  new cb::CommonARLibsVersionsStateControllerLibARCommandsVersion(
    nh, priv_nh, "states/common/ARLibsVersionsState/ControllerLibARCommandsVersion"));
common_arlibsversionsstate_skycontrollerlibarcommandsversion_ptr.reset(
  new cb::CommonARLibsVersionsStateSkyControllerLibARCommandsVersion(
    nh, priv_nh, "states/common/ARLibsVersionsState/SkyControllerLibARCommandsVersion"));
common_arlibsversionsstate_devicelibarcommandsversion_ptr.reset(
  new cb::CommonARLibsVersionsStateDeviceLibARCommandsVersion(
    nh, priv_nh, "states/common/ARLibsVersionsState/DeviceLibARCommandsVersion"));
common_audiostate_audiostreamingrunning_ptr.reset(
  new cb::CommonAudioStateAudioStreamingRunning(
    nh, priv_nh, "states/common/AudioState/AudioStreamingRunning"));
common_headlightsstate_intensitychanged_ptr.reset(
  new cb::CommonHeadlightsStateintensityChanged(
    nh, priv_nh, "states/common/HeadlightsState/intensityChanged"));
common_animationsstate_list_ptr.reset(
  new cb::CommonAnimationsStateList(
    nh, priv_nh, "states/common/AnimationsState/List"));
common_accessorystate_supportedaccessorieslistchanged_ptr.reset(
  new cb::CommonAccessoryStateSupportedAccessoriesListChanged(
    nh, priv_nh, "states/common/AccessoryState/SupportedAccessoriesListChanged"));
common_accessorystate_accessoryconfigchanged_ptr.reset(
  new cb::CommonAccessoryStateAccessoryConfigChanged(
    nh, priv_nh, "states/common/AccessoryState/AccessoryConfigChanged"));
common_accessorystate_accessoryconfigmodificationenabled_ptr.reset(
  new cb::CommonAccessoryStateAccessoryConfigModificationEnabled(
    nh, priv_nh, "states/common/AccessoryState/AccessoryConfigModificationEnabled"));
common_chargerstate_maxchargeratechanged_ptr.reset(
  new cb::CommonChargerStateMaxChargeRateChanged(
    nh, priv_nh, "states/common/ChargerState/MaxChargeRateChanged"));
common_chargerstate_currentchargestatechanged_ptr.reset(
  new cb::CommonChargerStateCurrentChargeStateChanged(
    nh, priv_nh, "states/common/ChargerState/CurrentChargeStateChanged"));
common_chargerstate_lastchargeratechanged_ptr.reset(
  new cb::CommonChargerStateLastChargeRateChanged(
    nh, priv_nh, "states/common/ChargerState/LastChargeRateChanged"));
common_chargerstate_charginginfo_ptr.reset(
  new cb::CommonChargerStateChargingInfo(
    nh, priv_nh, "states/common/ChargerState/ChargingInfo"));
common_runstate_runidchanged_ptr.reset(
  new cb::CommonRunStateRunIdChanged(
    nh, priv_nh, "states/common/RunState/RunIdChanged"));

// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_allstateschanged_ptr->GetCommandKey(),
                      common_commonstate_allstateschanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_batterystatechanged_ptr->GetCommandKey(),
                      common_commonstate_batterystatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_massstoragestatelistchanged_ptr->GetCommandKey(),
                      common_commonstate_massstoragestatelistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_massstorageinfostatelistchanged_ptr->GetCommandKey(),
                      common_commonstate_massstorageinfostatelistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_currentdatechanged_ptr->GetCommandKey(),
                      common_commonstate_currentdatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_currenttimechanged_ptr->GetCommandKey(),
                      common_commonstate_currenttimechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_massstorageinforemaininglistchanged_ptr->GetCommandKey(),
                      common_commonstate_massstorageinforemaininglistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_wifisignalchanged_ptr->GetCommandKey(),
                      common_commonstate_wifisignalchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_sensorsstateslistchanged_ptr->GetCommandKey(),
                      common_commonstate_sensorsstateslistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_productmodel_ptr->GetCommandKey(),
                      common_commonstate_productmodel_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_countrylistknown_ptr->GetCommandKey(),
                      common_commonstate_countrylistknown_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_deprecatedmassstoragecontentchanged_ptr->GetCommandKey(),
                      common_commonstate_deprecatedmassstoragecontentchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_massstoragecontent_ptr->GetCommandKey(),
                      common_commonstate_massstoragecontent_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_massstoragecontentforcurrentrun_ptr->GetCommandKey(),
                      common_commonstate_massstoragecontentforcurrentrun_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_commonstate_videorecordingtimestamp_ptr->GetCommandKey(),
                      common_commonstate_videorecordingtimestamp_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_overheatstate_overheatchanged_ptr->GetCommandKey(),
                      common_overheatstate_overheatchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_overheatstate_overheatregulationchanged_ptr->GetCommandKey(),
                      common_overheatstate_overheatregulationchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_mavlinkstate_mavlinkfileplayingstatechanged_ptr->GetCommandKey(),
                      common_mavlinkstate_mavlinkfileplayingstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_mavlinkstate_mavlinkplayerrorstatechanged_ptr->GetCommandKey(),
                      common_mavlinkstate_mavlinkplayerrorstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_mavlinkstate_missionitemexecuted_ptr->GetCommandKey(),
                      common_mavlinkstate_missionitemexecuted_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_calibrationstate_magnetocalibrationstatechanged_ptr->GetCommandKey(),
                      common_calibrationstate_magnetocalibrationstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_calibrationstate_magnetocalibrationrequiredstate_ptr->GetCommandKey(),
                      common_calibrationstate_magnetocalibrationrequiredstate_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_calibrationstate_magnetocalibrationaxistocalibratechanged_ptr->GetCommandKey(),
                      common_calibrationstate_magnetocalibrationaxistocalibratechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_calibrationstate_magnetocalibrationstartedchanged_ptr->GetCommandKey(),
                      common_calibrationstate_magnetocalibrationstartedchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_calibrationstate_pitotcalibrationstatechanged_ptr->GetCommandKey(),
                      common_calibrationstate_pitotcalibrationstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_flightplanstate_availabilitystatechanged_ptr->GetCommandKey(),
                      common_flightplanstate_availabilitystatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_flightplanstate_componentstatelistchanged_ptr->GetCommandKey(),
                      common_flightplanstate_componentstatelistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_flightplanstate_lockstatechanged_ptr->GetCommandKey(),
                      common_flightplanstate_lockstatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_arlibsversionsstate_controllerlibarcommandsversion_ptr->GetCommandKey(),
                      common_arlibsversionsstate_controllerlibarcommandsversion_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_arlibsversionsstate_skycontrollerlibarcommandsversion_ptr->GetCommandKey(),
                      common_arlibsversionsstate_skycontrollerlibarcommandsversion_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_arlibsversionsstate_devicelibarcommandsversion_ptr->GetCommandKey(),
                      common_arlibsversionsstate_devicelibarcommandsversion_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_audiostate_audiostreamingrunning_ptr->GetCommandKey(),
                      common_audiostate_audiostreamingrunning_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_headlightsstate_intensitychanged_ptr->GetCommandKey(),
                      common_headlightsstate_intensitychanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_animationsstate_list_ptr->GetCommandKey(),
                      common_animationsstate_list_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_accessorystate_supportedaccessorieslistchanged_ptr->GetCommandKey(),
                      common_accessorystate_supportedaccessorieslistchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_accessorystate_accessoryconfigchanged_ptr->GetCommandKey(),
                      common_accessorystate_accessoryconfigchanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_accessorystate_accessoryconfigmodificationenabled_ptr->GetCommandKey(),
                      common_accessorystate_accessoryconfigmodificationenabled_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_chargerstate_maxchargeratechanged_ptr->GetCommandKey(),
                      common_chargerstate_maxchargeratechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_chargerstate_currentchargestatechanged_ptr->GetCommandKey(),
                      common_chargerstate_currentchargestatechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_chargerstate_lastchargeratechanged_ptr->GetCommandKey(),
                      common_chargerstate_lastchargeratechanged_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_chargerstate_charginginfo_ptr->GetCommandKey(),
                      common_chargerstate_charginginfo_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      common_runstate_runidchanged_ptr->GetCommandKey(),
                      common_runstate_runidchanged_ptr));
#endif  // UPDTAE_CALLBACK_MAP
