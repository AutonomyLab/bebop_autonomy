 .. common_states_param_topic.rst
 .. auto-generated from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/ab28dab91845cd36c4d7002b55f70805deaff3c8/xml/common.xml
 .. Do not modify this file by hand. Check scripts/meta folder for generator files.

*****************************************************************************************
List of common States and Corresponding ROS Parameters and Topics
*****************************************************************************************

`CommonCommonStateAllStatesChanged`_
  All states have been sent.\n\n **Please note that you should not care about this event if you are using the libARController API as this library is handling the connection process for you.**
`CommonCommonStateBatteryStateChanged`_
  Battery state.
`CommonCommonStateMassStorageStateListChanged`_
  Mass storage state list.
`CommonCommonStateMassStorageInfoStateListChanged`_
  Mass storage info state list.
`CommonCommonStateCurrentDateChanged`_
  Date changed.\n Corresponds to the latest date set on the drone.\n\n **Please note that you should not care about this event if you are using the libARController API as this library is handling the connection process for you.**
`CommonCommonStateCurrentTimeChanged`_
  Time changed.\n Corresponds to the latest time set on the drone.\n\n **Please note that you should not care about this event if you are using the libARController API as this library is handling the connection process for you.**
`CommonCommonStateMassStorageInfoRemainingListChanged`_
  Mass storage remaining data list.
`CommonCommonStateWifiSignalChanged`_
  Rssi (Wifi Signal between controller and product) changed.
`CommonCommonStateSensorsStatesListChanged`_
  Sensors state list.
`CommonCommonStateProductModel`_
  Product sub-model.\n This can be used to customize the UI depending on the product.
`CommonCommonStateCountryListKnown`_
  List of countries known by the drone.
`CommonCommonStateDeprecatedMassStorageContentChanged`_
  Mass storage content changed.
`CommonCommonStateMassStorageContent`_
  Mass storage content.
`CommonCommonStateMassStorageContentForCurrentRun`_
  Mass storage content for current run.\n Only counts the files related to the current run (see [RunId](#0-30-0))
`CommonCommonStateVideoRecordingTimestamp`_
  Current or last video recording timestamp.\n Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970.\n **Please note that values dont persist after drone reboot**
`CommonOverHeatStateOverHeatChanged`_
  Overheat temperature reached.
`CommonOverHeatStateOverHeatRegulationChanged`_
  Overheat regulation type.
`CommonMavlinkStateMavlinkFilePlayingStateChanged`_
  Playing state of a FlightPlan.
`CommonMavlinkStateMavlinkPlayErrorStateChanged`_
  FlightPlan error.
`CommonMavlinkStateMissionItemExecuted`_
  Mission item has been executed.
`CommonCalibrationStateMagnetoCalibrationStateChanged`_
  Magneto calib process axis state.
`CommonCalibrationStateMagnetoCalibrationRequiredState`_
  Calibration required.
`CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged`_
  Axis to calibrate during calibration process.
`CommonCalibrationStateMagnetoCalibrationStartedChanged`_
  Calibration process state.
`CommonCalibrationStatePitotCalibrationStateChanged`_
  
`CommonFlightPlanStateAvailabilityStateChanged`_
  FlightPlan availability.\n Availability is linked to GPS fix, magnetometer calibration, sensor states...
`CommonFlightPlanStateComponentStateListChanged`_
  FlightPlan components state list.
`CommonFlightPlanStateLockStateChanged`_
  FlightPlan lock state.\n Represents the fact that the controller is able or not to stop or pause a playing FlightPlan
`CommonARLibsVersionsStateControllerLibARCommandsVersion`_
  
`CommonARLibsVersionsStateSkyControllerLibARCommandsVersion`_
  
`CommonARLibsVersionsStateDeviceLibARCommandsVersion`_
  
`CommonAudioStateAudioStreamingRunning`_
  Audio stream direction.
`CommonHeadlightsStateintensityChanged`_
  Lighting LEDs intensity.
`CommonAnimationsStateList`_
  Paramaterless animations state list.
`CommonAccessoryStateSupportedAccessoriesListChanged`_
  Supported accessories list.
`CommonAccessoryStateAccessoryConfigChanged`_
  Accessory config.
`CommonAccessoryStateAccessoryConfigModificationEnabled`_
  Availability to declare or not an accessory.
`CommonChargerStateMaxChargeRateChanged`_
  Max charge rate.
`CommonChargerStateCurrentChargeStateChanged`_
  Current charge state.
`CommonChargerStateLastChargeRateChanged`_
  Last charge rate.
`CommonChargerStateChargingInfo`_
  Charging information.
`CommonRunStateRunIdChanged`_
  Current run id.\n A run id is uniquely identifying a run or a flight.\n For each run is generated on the drone a file which can be used by Academy to sum up the run.\n Also, each medias taken during a run has a filename containing the run id.

CommonCommonStateAllStatesChanged
####################################################################################
All states have been sent.\n\n **Please note that you should not care about this event if you are using the libARController API as this library is handling the connection process for you.**

- Parameter: ``~states/enable_commonstate_allstateschanged``
- Topic: ``states/common/CommonState/AllStatesChanged``
- Message type: ``bebop_msgs::CommonCommonStateAllStatesChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateAllStatesChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateAllStatesChanged.msg
  :name: CommonCommonStateAllStatesChanged_msg

CommonCommonStateBatteryStateChanged
####################################################################################
Battery state.

- Parameter: ``~states/enable_commonstate_batterystatechanged``
- Topic: ``states/common/CommonState/BatteryStateChanged``
- Message type: ``bebop_msgs::CommonCommonStateBatteryStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateBatteryStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateBatteryStateChanged.msg
  :name: CommonCommonStateBatteryStateChanged_msg

CommonCommonStateMassStorageStateListChanged
####################################################################################
Mass storage state list.

- Parameter: ``~states/enable_commonstate_massstoragestatelistchanged``
- Topic: ``states/common/CommonState/MassStorageStateListChanged``
- Message type: ``bebop_msgs::CommonCommonStateMassStorageStateListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateMassStorageStateListChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateMassStorageStateListChanged.msg
  :name: CommonCommonStateMassStorageStateListChanged_msg

CommonCommonStateMassStorageInfoStateListChanged
####################################################################################
Mass storage info state list.

- Parameter: ``~states/enable_commonstate_massstorageinfostatelistchanged``
- Topic: ``states/common/CommonState/MassStorageInfoStateListChanged``
- Message type: ``bebop_msgs::CommonCommonStateMassStorageInfoStateListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateMassStorageInfoStateListChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateMassStorageInfoStateListChanged.msg
  :name: CommonCommonStateMassStorageInfoStateListChanged_msg

CommonCommonStateCurrentDateChanged
####################################################################################
Date changed.\n Corresponds to the latest date set on the drone.\n\n **Please note that you should not care about this event if you are using the libARController API as this library is handling the connection process for you.**

- Parameter: ``~states/enable_commonstate_currentdatechanged``
- Topic: ``states/common/CommonState/CurrentDateChanged``
- Message type: ``bebop_msgs::CommonCommonStateCurrentDateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateCurrentDateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateCurrentDateChanged.msg
  :name: CommonCommonStateCurrentDateChanged_msg

CommonCommonStateCurrentTimeChanged
####################################################################################
Time changed.\n Corresponds to the latest time set on the drone.\n\n **Please note that you should not care about this event if you are using the libARController API as this library is handling the connection process for you.**

- Parameter: ``~states/enable_commonstate_currenttimechanged``
- Topic: ``states/common/CommonState/CurrentTimeChanged``
- Message type: ``bebop_msgs::CommonCommonStateCurrentTimeChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateCurrentTimeChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateCurrentTimeChanged.msg
  :name: CommonCommonStateCurrentTimeChanged_msg

CommonCommonStateMassStorageInfoRemainingListChanged
####################################################################################
Mass storage remaining data list.

- Parameter: ``~states/enable_commonstate_massstorageinforemaininglistchanged``
- Topic: ``states/common/CommonState/MassStorageInfoRemainingListChanged``
- Message type: ``bebop_msgs::CommonCommonStateMassStorageInfoRemainingListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateMassStorageInfoRemainingListChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateMassStorageInfoRemainingListChanged.msg
  :name: CommonCommonStateMassStorageInfoRemainingListChanged_msg

CommonCommonStateWifiSignalChanged
####################################################################################
Rssi (Wifi Signal between controller and product) changed.

- Parameter: ``~states/enable_commonstate_wifisignalchanged``
- Topic: ``states/common/CommonState/WifiSignalChanged``
- Message type: ``bebop_msgs::CommonCommonStateWifiSignalChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateWifiSignalChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateWifiSignalChanged.msg
  :name: CommonCommonStateWifiSignalChanged_msg

CommonCommonStateSensorsStatesListChanged
####################################################################################
Sensors state list.

- Parameter: ``~states/enable_commonstate_sensorsstateslistchanged``
- Topic: ``states/common/CommonState/SensorsStatesListChanged``
- Message type: ``bebop_msgs::CommonCommonStateSensorsStatesListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateSensorsStatesListChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateSensorsStatesListChanged.msg
  :name: CommonCommonStateSensorsStatesListChanged_msg

CommonCommonStateProductModel
####################################################################################
Product sub-model.\n This can be used to customize the UI depending on the product.

- Parameter: ``~states/enable_commonstate_productmodel``
- Topic: ``states/common/CommonState/ProductModel``
- Message type: ``bebop_msgs::CommonCommonStateProductModel``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateProductModel.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateProductModel.msg
  :name: CommonCommonStateProductModel_msg

CommonCommonStateCountryListKnown
####################################################################################
List of countries known by the drone.

- Parameter: ``~states/enable_commonstate_countrylistknown``
- Topic: ``states/common/CommonState/CountryListKnown``
- Message type: ``bebop_msgs::CommonCommonStateCountryListKnown``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateCountryListKnown.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateCountryListKnown.msg
  :name: CommonCommonStateCountryListKnown_msg

CommonCommonStateDeprecatedMassStorageContentChanged
####################################################################################
Mass storage content changed.

- Parameter: ``~states/enable_commonstate_deprecatedmassstoragecontentchanged``
- Topic: ``states/common/CommonState/DeprecatedMassStorageContentChanged``
- Message type: ``bebop_msgs::CommonCommonStateDeprecatedMassStorageContentChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateDeprecatedMassStorageContentChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateDeprecatedMassStorageContentChanged.msg
  :name: CommonCommonStateDeprecatedMassStorageContentChanged_msg

CommonCommonStateMassStorageContent
####################################################################################
Mass storage content.

- Parameter: ``~states/enable_commonstate_massstoragecontent``
- Topic: ``states/common/CommonState/MassStorageContent``
- Message type: ``bebop_msgs::CommonCommonStateMassStorageContent``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateMassStorageContent.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateMassStorageContent.msg
  :name: CommonCommonStateMassStorageContent_msg

CommonCommonStateMassStorageContentForCurrentRun
####################################################################################
Mass storage content for current run.\n Only counts the files related to the current run (see [RunId](#0-30-0))

- Parameter: ``~states/enable_commonstate_massstoragecontentforcurrentrun``
- Topic: ``states/common/CommonState/MassStorageContentForCurrentRun``
- Message type: ``bebop_msgs::CommonCommonStateMassStorageContentForCurrentRun``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateMassStorageContentForCurrentRun.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateMassStorageContentForCurrentRun.msg
  :name: CommonCommonStateMassStorageContentForCurrentRun_msg

CommonCommonStateVideoRecordingTimestamp
####################################################################################
Current or last video recording timestamp.\n Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970.\n **Please note that values dont persist after drone reboot**

- Parameter: ``~states/enable_commonstate_videorecordingtimestamp``
- Topic: ``states/common/CommonState/VideoRecordingTimestamp``
- Message type: ``bebop_msgs::CommonCommonStateVideoRecordingTimestamp``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCommonStateVideoRecordingTimestamp.msg
  :lines: 8-
  :language: python
  :caption: CommonCommonStateVideoRecordingTimestamp.msg
  :name: CommonCommonStateVideoRecordingTimestamp_msg

CommonOverHeatStateOverHeatChanged
####################################################################################
Overheat temperature reached.

- Parameter: ``~states/enable_overheatstate_overheatchanged``
- Topic: ``states/common/OverHeatState/OverHeatChanged``
- Message type: ``bebop_msgs::CommonOverHeatStateOverHeatChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonOverHeatStateOverHeatChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonOverHeatStateOverHeatChanged.msg
  :name: CommonOverHeatStateOverHeatChanged_msg

CommonOverHeatStateOverHeatRegulationChanged
####################################################################################
Overheat regulation type.

- Parameter: ``~states/enable_overheatstate_overheatregulationchanged``
- Topic: ``states/common/OverHeatState/OverHeatRegulationChanged``
- Message type: ``bebop_msgs::CommonOverHeatStateOverHeatRegulationChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonOverHeatStateOverHeatRegulationChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonOverHeatStateOverHeatRegulationChanged.msg
  :name: CommonOverHeatStateOverHeatRegulationChanged_msg

CommonMavlinkStateMavlinkFilePlayingStateChanged
####################################################################################
Playing state of a FlightPlan.

- Parameter: ``~states/enable_mavlinkstate_mavlinkfileplayingstatechanged``
- Topic: ``states/common/MavlinkState/MavlinkFilePlayingStateChanged``
- Message type: ``bebop_msgs::CommonMavlinkStateMavlinkFilePlayingStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonMavlinkStateMavlinkFilePlayingStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonMavlinkStateMavlinkFilePlayingStateChanged.msg
  :name: CommonMavlinkStateMavlinkFilePlayingStateChanged_msg

CommonMavlinkStateMavlinkPlayErrorStateChanged
####################################################################################
FlightPlan error.

- Parameter: ``~states/enable_mavlinkstate_mavlinkplayerrorstatechanged``
- Topic: ``states/common/MavlinkState/MavlinkPlayErrorStateChanged``
- Message type: ``bebop_msgs::CommonMavlinkStateMavlinkPlayErrorStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonMavlinkStateMavlinkPlayErrorStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonMavlinkStateMavlinkPlayErrorStateChanged.msg
  :name: CommonMavlinkStateMavlinkPlayErrorStateChanged_msg

CommonMavlinkStateMissionItemExecuted
####################################################################################
Mission item has been executed.

- Parameter: ``~states/enable_mavlinkstate_missionitemexecuted``
- Topic: ``states/common/MavlinkState/MissionItemExecuted``
- Message type: ``bebop_msgs::CommonMavlinkStateMissionItemExecuted``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonMavlinkStateMissionItemExecuted.msg
  :lines: 8-
  :language: python
  :caption: CommonMavlinkStateMissionItemExecuted.msg
  :name: CommonMavlinkStateMissionItemExecuted_msg

CommonCalibrationStateMagnetoCalibrationStateChanged
####################################################################################
Magneto calib process axis state.

- Parameter: ``~states/enable_calibrationstate_magnetocalibrationstatechanged``
- Topic: ``states/common/CalibrationState/MagnetoCalibrationStateChanged``
- Message type: ``bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCalibrationStateMagnetoCalibrationStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCalibrationStateMagnetoCalibrationStateChanged.msg
  :name: CommonCalibrationStateMagnetoCalibrationStateChanged_msg

CommonCalibrationStateMagnetoCalibrationRequiredState
####################################################################################
Calibration required.

- Parameter: ``~states/enable_calibrationstate_magnetocalibrationrequiredstate``
- Topic: ``states/common/CalibrationState/MagnetoCalibrationRequiredState``
- Message type: ``bebop_msgs::CommonCalibrationStateMagnetoCalibrationRequiredState``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCalibrationStateMagnetoCalibrationRequiredState.msg
  :lines: 8-
  :language: python
  :caption: CommonCalibrationStateMagnetoCalibrationRequiredState.msg
  :name: CommonCalibrationStateMagnetoCalibrationRequiredState_msg

CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged
####################################################################################
Axis to calibrate during calibration process.

- Parameter: ``~states/enable_calibrationstate_magnetocalibrationaxistocalibratechanged``
- Topic: ``states/common/CalibrationState/MagnetoCalibrationAxisToCalibrateChanged``
- Message type: ``bebop_msgs::CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged.msg
  :name: CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged_msg

CommonCalibrationStateMagnetoCalibrationStartedChanged
####################################################################################
Calibration process state.

- Parameter: ``~states/enable_calibrationstate_magnetocalibrationstartedchanged``
- Topic: ``states/common/CalibrationState/MagnetoCalibrationStartedChanged``
- Message type: ``bebop_msgs::CommonCalibrationStateMagnetoCalibrationStartedChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCalibrationStateMagnetoCalibrationStartedChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCalibrationStateMagnetoCalibrationStartedChanged.msg
  :name: CommonCalibrationStateMagnetoCalibrationStartedChanged_msg

CommonCalibrationStatePitotCalibrationStateChanged
####################################################################################


- Parameter: ``~states/enable_calibrationstate_pitotcalibrationstatechanged``
- Topic: ``states/common/CalibrationState/PitotCalibrationStateChanged``
- Message type: ``bebop_msgs::CommonCalibrationStatePitotCalibrationStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonCalibrationStatePitotCalibrationStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonCalibrationStatePitotCalibrationStateChanged.msg
  :name: CommonCalibrationStatePitotCalibrationStateChanged_msg

CommonFlightPlanStateAvailabilityStateChanged
####################################################################################
FlightPlan availability.\n Availability is linked to GPS fix, magnetometer calibration, sensor states...

- Parameter: ``~states/enable_flightplanstate_availabilitystatechanged``
- Topic: ``states/common/FlightPlanState/AvailabilityStateChanged``
- Message type: ``bebop_msgs::CommonFlightPlanStateAvailabilityStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonFlightPlanStateAvailabilityStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonFlightPlanStateAvailabilityStateChanged.msg
  :name: CommonFlightPlanStateAvailabilityStateChanged_msg

CommonFlightPlanStateComponentStateListChanged
####################################################################################
FlightPlan components state list.

- Parameter: ``~states/enable_flightplanstate_componentstatelistchanged``
- Topic: ``states/common/FlightPlanState/ComponentStateListChanged``
- Message type: ``bebop_msgs::CommonFlightPlanStateComponentStateListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonFlightPlanStateComponentStateListChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonFlightPlanStateComponentStateListChanged.msg
  :name: CommonFlightPlanStateComponentStateListChanged_msg

CommonFlightPlanStateLockStateChanged
####################################################################################
FlightPlan lock state.\n Represents the fact that the controller is able or not to stop or pause a playing FlightPlan

- Parameter: ``~states/enable_flightplanstate_lockstatechanged``
- Topic: ``states/common/FlightPlanState/LockStateChanged``
- Message type: ``bebop_msgs::CommonFlightPlanStateLockStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonFlightPlanStateLockStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonFlightPlanStateLockStateChanged.msg
  :name: CommonFlightPlanStateLockStateChanged_msg

CommonARLibsVersionsStateControllerLibARCommandsVersion
####################################################################################


- Parameter: ``~states/enable_arlibsversionsstate_controllerlibarcommandsversion``
- Topic: ``states/common/ARLibsVersionsState/ControllerLibARCommandsVersion``
- Message type: ``bebop_msgs::CommonARLibsVersionsStateControllerLibARCommandsVersion``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonARLibsVersionsStateControllerLibARCommandsVersion.msg
  :lines: 8-
  :language: python
  :caption: CommonARLibsVersionsStateControllerLibARCommandsVersion.msg
  :name: CommonARLibsVersionsStateControllerLibARCommandsVersion_msg

CommonARLibsVersionsStateSkyControllerLibARCommandsVersion
####################################################################################


- Parameter: ``~states/enable_arlibsversionsstate_skycontrollerlibarcommandsversion``
- Topic: ``states/common/ARLibsVersionsState/SkyControllerLibARCommandsVersion``
- Message type: ``bebop_msgs::CommonARLibsVersionsStateSkyControllerLibARCommandsVersion``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonARLibsVersionsStateSkyControllerLibARCommandsVersion.msg
  :lines: 8-
  :language: python
  :caption: CommonARLibsVersionsStateSkyControllerLibARCommandsVersion.msg
  :name: CommonARLibsVersionsStateSkyControllerLibARCommandsVersion_msg

CommonARLibsVersionsStateDeviceLibARCommandsVersion
####################################################################################


- Parameter: ``~states/enable_arlibsversionsstate_devicelibarcommandsversion``
- Topic: ``states/common/ARLibsVersionsState/DeviceLibARCommandsVersion``
- Message type: ``bebop_msgs::CommonARLibsVersionsStateDeviceLibARCommandsVersion``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonARLibsVersionsStateDeviceLibARCommandsVersion.msg
  :lines: 8-
  :language: python
  :caption: CommonARLibsVersionsStateDeviceLibARCommandsVersion.msg
  :name: CommonARLibsVersionsStateDeviceLibARCommandsVersion_msg

CommonAudioStateAudioStreamingRunning
####################################################################################
Audio stream direction.

- Parameter: ``~states/enable_audiostate_audiostreamingrunning``
- Topic: ``states/common/AudioState/AudioStreamingRunning``
- Message type: ``bebop_msgs::CommonAudioStateAudioStreamingRunning``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonAudioStateAudioStreamingRunning.msg
  :lines: 8-
  :language: python
  :caption: CommonAudioStateAudioStreamingRunning.msg
  :name: CommonAudioStateAudioStreamingRunning_msg

CommonHeadlightsStateintensityChanged
####################################################################################
Lighting LEDs intensity.

- Parameter: ``~states/enable_headlightsstate_intensitychanged``
- Topic: ``states/common/HeadlightsState/intensityChanged``
- Message type: ``bebop_msgs::CommonHeadlightsStateintensityChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonHeadlightsStateintensityChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonHeadlightsStateintensityChanged.msg
  :name: CommonHeadlightsStateintensityChanged_msg

CommonAnimationsStateList
####################################################################################
Paramaterless animations state list.

- Parameter: ``~states/enable_animationsstate_list``
- Topic: ``states/common/AnimationsState/List``
- Message type: ``bebop_msgs::CommonAnimationsStateList``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonAnimationsStateList.msg
  :lines: 8-
  :language: python
  :caption: CommonAnimationsStateList.msg
  :name: CommonAnimationsStateList_msg

CommonAccessoryStateSupportedAccessoriesListChanged
####################################################################################
Supported accessories list.

- Parameter: ``~states/enable_accessorystate_supportedaccessorieslistchanged``
- Topic: ``states/common/AccessoryState/SupportedAccessoriesListChanged``
- Message type: ``bebop_msgs::CommonAccessoryStateSupportedAccessoriesListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonAccessoryStateSupportedAccessoriesListChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonAccessoryStateSupportedAccessoriesListChanged.msg
  :name: CommonAccessoryStateSupportedAccessoriesListChanged_msg

CommonAccessoryStateAccessoryConfigChanged
####################################################################################
Accessory config.

- Parameter: ``~states/enable_accessorystate_accessoryconfigchanged``
- Topic: ``states/common/AccessoryState/AccessoryConfigChanged``
- Message type: ``bebop_msgs::CommonAccessoryStateAccessoryConfigChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonAccessoryStateAccessoryConfigChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonAccessoryStateAccessoryConfigChanged.msg
  :name: CommonAccessoryStateAccessoryConfigChanged_msg

CommonAccessoryStateAccessoryConfigModificationEnabled
####################################################################################
Availability to declare or not an accessory.

- Parameter: ``~states/enable_accessorystate_accessoryconfigmodificationenabled``
- Topic: ``states/common/AccessoryState/AccessoryConfigModificationEnabled``
- Message type: ``bebop_msgs::CommonAccessoryStateAccessoryConfigModificationEnabled``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonAccessoryStateAccessoryConfigModificationEnabled.msg
  :lines: 8-
  :language: python
  :caption: CommonAccessoryStateAccessoryConfigModificationEnabled.msg
  :name: CommonAccessoryStateAccessoryConfigModificationEnabled_msg

CommonChargerStateMaxChargeRateChanged
####################################################################################
Max charge rate.

- Parameter: ``~states/enable_chargerstate_maxchargeratechanged``
- Topic: ``states/common/ChargerState/MaxChargeRateChanged``
- Message type: ``bebop_msgs::CommonChargerStateMaxChargeRateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonChargerStateMaxChargeRateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonChargerStateMaxChargeRateChanged.msg
  :name: CommonChargerStateMaxChargeRateChanged_msg

CommonChargerStateCurrentChargeStateChanged
####################################################################################
Current charge state.

- Parameter: ``~states/enable_chargerstate_currentchargestatechanged``
- Topic: ``states/common/ChargerState/CurrentChargeStateChanged``
- Message type: ``bebop_msgs::CommonChargerStateCurrentChargeStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonChargerStateCurrentChargeStateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonChargerStateCurrentChargeStateChanged.msg
  :name: CommonChargerStateCurrentChargeStateChanged_msg

CommonChargerStateLastChargeRateChanged
####################################################################################
Last charge rate.

- Parameter: ``~states/enable_chargerstate_lastchargeratechanged``
- Topic: ``states/common/ChargerState/LastChargeRateChanged``
- Message type: ``bebop_msgs::CommonChargerStateLastChargeRateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonChargerStateLastChargeRateChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonChargerStateLastChargeRateChanged.msg
  :name: CommonChargerStateLastChargeRateChanged_msg

CommonChargerStateChargingInfo
####################################################################################
Charging information.

- Parameter: ``~states/enable_chargerstate_charginginfo``
- Topic: ``states/common/ChargerState/ChargingInfo``
- Message type: ``bebop_msgs::CommonChargerStateChargingInfo``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonChargerStateChargingInfo.msg
  :lines: 8-
  :language: python
  :caption: CommonChargerStateChargingInfo.msg
  :name: CommonChargerStateChargingInfo_msg

CommonRunStateRunIdChanged
####################################################################################
Current run id.\n A run id is uniquely identifying a run or a flight.\n For each run is generated on the drone a file which can be used by Academy to sum up the run.\n Also, each medias taken during a run has a filename containing the run id.

- Parameter: ``~states/enable_runstate_runidchanged``
- Topic: ``states/common/RunState/RunIdChanged``
- Message type: ``bebop_msgs::CommonRunStateRunIdChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/CommonRunStateRunIdChanged.msg
  :lines: 8-
  :language: python
  :caption: CommonRunStateRunIdChanged.msg
  :name: CommonRunStateRunIdChanged_msg

