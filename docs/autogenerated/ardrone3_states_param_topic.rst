 .. ardrone3_states_param_topic.rst
 .. auto-generated from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/ab28dab91845cd36c4d7002b55f70805deaff3c8/xml/ardrone3.xml
 .. Do not modify this file by hand. Check scripts/meta folder for generator files.

*****************************************************************************************
List of ardrone3 States and Corresponding ROS Parameters and Topics
*****************************************************************************************

`Ardrone3MediaRecordStatePictureStateChanged`_
  Picture state.
`Ardrone3MediaRecordStateVideoStateChanged`_
  Picture record state.
`Ardrone3MediaRecordStatePictureStateChangedV2`_
  Picture state.
`Ardrone3MediaRecordStateVideoStateChangedV2`_
  Video record state.
`Ardrone3MediaRecordStateVideoResolutionState`_
  Video resolution.\n Informs about streaming and recording video resolutions.\n Note that this is only an indication about what the resolution should be. To know the real resolution, you should get it from the frame.
`Ardrone3PilotingStateFlatTrimChanged`_
  Drone acknowledges that flat trim was correctly processed.
`Ardrone3PilotingStateFlyingStateChanged`_
  Flying state.
`Ardrone3PilotingStateAlertStateChanged`_
  Alert state.
`Ardrone3PilotingStateNavigateHomeStateChanged`_
  Return home state.\n Availability is related to gps fix, magnetometer calibration.
`Ardrone3PilotingStatePositionChanged`_
  Drones position changed.
`Ardrone3PilotingStateSpeedChanged`_
  Drones speed changed.\n Expressed in the NED referential (North-East-Down).
`Ardrone3PilotingStateAttitudeChanged`_
  Drones attitude changed.
`Ardrone3PilotingStateAutoTakeOffModeChanged`_
  Auto takeoff mode
`Ardrone3PilotingStateAltitudeChanged`_
  Drones altitude changed.\n The altitude reported is the altitude above the take off point.\n To get the altitude above sea level, see [PositionChanged](#1-4-4).
`Ardrone3PilotingStateGpsLocationChanged`_
  Drones location changed.\n This event is meant to replace [PositionChanged](#1-4-4).
`Ardrone3PilotingStateLandingStateChanged`_
  Landing state.\n Only available for fixed wings (which have two landing modes).
`Ardrone3PilotingStateAirSpeedChanged`_
  Drones air speed changed\n Expressed in the drones referential.
`Ardrone3PilotingStatemoveToChanged`_
  The drone moves or moved to a given location.
`Ardrone3NetworkStateWifiScanListChanged`_
  Wifi scan results.\n Please note that the list is not complete until you receive the event [WifiScanEnded](#1-14-1).
`Ardrone3NetworkStateAllWifiScanChanged`_
  Wifi scan ended.\n When receiving this event, the list of [WifiScanResults](#1-14-0) is complete.
`Ardrone3NetworkStateWifiAuthChannelListChanged`_
  Available wifi channels.\n Please note that the list is not complete until you receive the event [AvailableWifiChannelsCompleted](#1-14-3).
`Ardrone3NetworkStateAllWifiAuthChannelChanged`_
  Available wifi channels completed.\n When receiving this event, the list of [AvailableWifiChannels](#1-14-2) is complete.
`Ardrone3MediaStreamingStateVideoEnableChanged`_
  Video stream state.
`Ardrone3MediaStreamingStateVideoStreamModeChanged`_
  
`Ardrone3CameraStateOrientation`_
  Camera orientation.
`Ardrone3CameraStatedefaultCameraOrientation`_
  Orientation of the center of the camera.\n This is the value to send when you want to center the camera.
`Ardrone3CameraStateOrientationV2`_
  Camera orientation with float arguments.
`Ardrone3CameraStatedefaultCameraOrientationV2`_
  Orientation of the center of the camera.\n This is the value to send when you want to center the camera.
`Ardrone3CameraStateVelocityRange`_
  Camera Orientation velocity limits.
`Ardrone3AntiflickeringStateelectricFrequencyChanged`_
  Electric frequency.\n This piece of information is used for the antiflickering when the [AntiflickeringMode](#1-30-1) is set to *auto*.
`Ardrone3AntiflickeringStatemodeChanged`_
  Antiflickering mode.
`Ardrone3GPSStateNumberOfSatelliteChanged`_
  Number of GPS satellites.
`Ardrone3GPSStateHomeTypeAvailabilityChanged`_
  Home type availability.
`Ardrone3GPSStateHomeTypeChosenChanged`_
  Home type.\n This choice is made by the drone, according to the [PreferredHomeType](#1-24-4) and the [HomeTypeAvailability](#1-31-1). The drone will choose the type matching with the user preference only if this type is available. If not, it will chose a type in this order:\n FOLLOWEE ; TAKEOFF ; PILOT ; FIRST_FIX
`Ardrone3PROStateFeatures`_
  Pro features.
`Ardrone3AccessoryStateConnectedAccessories`_
  List of all connected accessories. This event presents the list of all connected accessories. To actually use the component, use the component dedicated feature.

Ardrone3MediaRecordStatePictureStateChanged
####################################################################################
Picture state.

- Parameter: ``~states/enable_mediarecordstate_picturestatechanged``
- Topic: ``states/ardrone3/MediaRecordState/PictureStateChanged``
- Message type: ``bebop_msgs::Ardrone3MediaRecordStatePictureStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaRecordStatePictureStateChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaRecordStatePictureStateChanged.msg
  :name: Ardrone3MediaRecordStatePictureStateChanged_msg

Ardrone3MediaRecordStateVideoStateChanged
####################################################################################
Picture record state.

- Parameter: ``~states/enable_mediarecordstate_videostatechanged``
- Topic: ``states/ardrone3/MediaRecordState/VideoStateChanged``
- Message type: ``bebop_msgs::Ardrone3MediaRecordStateVideoStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaRecordStateVideoStateChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaRecordStateVideoStateChanged.msg
  :name: Ardrone3MediaRecordStateVideoStateChanged_msg

Ardrone3MediaRecordStatePictureStateChangedV2
####################################################################################
Picture state.

- Parameter: ``~states/enable_mediarecordstate_picturestatechangedv2``
- Topic: ``states/ardrone3/MediaRecordState/PictureStateChangedV2``
- Message type: ``bebop_msgs::Ardrone3MediaRecordStatePictureStateChangedV2``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaRecordStatePictureStateChangedV2.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaRecordStatePictureStateChangedV2.msg
  :name: Ardrone3MediaRecordStatePictureStateChangedV2_msg

Ardrone3MediaRecordStateVideoStateChangedV2
####################################################################################
Video record state.

- Parameter: ``~states/enable_mediarecordstate_videostatechangedv2``
- Topic: ``states/ardrone3/MediaRecordState/VideoStateChangedV2``
- Message type: ``bebop_msgs::Ardrone3MediaRecordStateVideoStateChangedV2``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaRecordStateVideoStateChangedV2.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaRecordStateVideoStateChangedV2.msg
  :name: Ardrone3MediaRecordStateVideoStateChangedV2_msg

Ardrone3MediaRecordStateVideoResolutionState
####################################################################################
Video resolution.\n Informs about streaming and recording video resolutions.\n Note that this is only an indication about what the resolution should be. To know the real resolution, you should get it from the frame.

- Parameter: ``~states/enable_mediarecordstate_videoresolutionstate``
- Topic: ``states/ardrone3/MediaRecordState/VideoResolutionState``
- Message type: ``bebop_msgs::Ardrone3MediaRecordStateVideoResolutionState``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaRecordStateVideoResolutionState.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaRecordStateVideoResolutionState.msg
  :name: Ardrone3MediaRecordStateVideoResolutionState_msg

Ardrone3PilotingStateFlatTrimChanged
####################################################################################
Drone acknowledges that flat trim was correctly processed.

- Parameter: ``~states/enable_pilotingstate_flattrimchanged``
- Topic: ``states/ardrone3/PilotingState/FlatTrimChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateFlatTrimChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateFlatTrimChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateFlatTrimChanged.msg
  :name: Ardrone3PilotingStateFlatTrimChanged_msg

Ardrone3PilotingStateFlyingStateChanged
####################################################################################
Flying state.

- Parameter: ``~states/enable_pilotingstate_flyingstatechanged``
- Topic: ``states/ardrone3/PilotingState/FlyingStateChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateFlyingStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateFlyingStateChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateFlyingStateChanged.msg
  :name: Ardrone3PilotingStateFlyingStateChanged_msg

Ardrone3PilotingStateAlertStateChanged
####################################################################################
Alert state.

- Parameter: ``~states/enable_pilotingstate_alertstatechanged``
- Topic: ``states/ardrone3/PilotingState/AlertStateChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateAlertStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateAlertStateChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateAlertStateChanged.msg
  :name: Ardrone3PilotingStateAlertStateChanged_msg

Ardrone3PilotingStateNavigateHomeStateChanged
####################################################################################
Return home state.\n Availability is related to gps fix, magnetometer calibration.

- Parameter: ``~states/enable_pilotingstate_navigatehomestatechanged``
- Topic: ``states/ardrone3/PilotingState/NavigateHomeStateChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateNavigateHomeStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateNavigateHomeStateChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateNavigateHomeStateChanged.msg
  :name: Ardrone3PilotingStateNavigateHomeStateChanged_msg

Ardrone3PilotingStatePositionChanged
####################################################################################
Drones position changed.

- Parameter: ``~states/enable_pilotingstate_positionchanged``
- Topic: ``states/ardrone3/PilotingState/PositionChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStatePositionChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStatePositionChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStatePositionChanged.msg
  :name: Ardrone3PilotingStatePositionChanged_msg

Ardrone3PilotingStateSpeedChanged
####################################################################################
Drones speed changed.\n Expressed in the NED referential (North-East-Down).

- Parameter: ``~states/enable_pilotingstate_speedchanged``
- Topic: ``states/ardrone3/PilotingState/SpeedChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateSpeedChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateSpeedChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateSpeedChanged.msg
  :name: Ardrone3PilotingStateSpeedChanged_msg

Ardrone3PilotingStateAttitudeChanged
####################################################################################
Drones attitude changed.

- Parameter: ``~states/enable_pilotingstate_attitudechanged``
- Topic: ``states/ardrone3/PilotingState/AttitudeChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateAttitudeChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateAttitudeChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateAttitudeChanged.msg
  :name: Ardrone3PilotingStateAttitudeChanged_msg

Ardrone3PilotingStateAutoTakeOffModeChanged
####################################################################################
Auto takeoff mode

- Parameter: ``~states/enable_pilotingstate_autotakeoffmodechanged``
- Topic: ``states/ardrone3/PilotingState/AutoTakeOffModeChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateAutoTakeOffModeChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateAutoTakeOffModeChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateAutoTakeOffModeChanged.msg
  :name: Ardrone3PilotingStateAutoTakeOffModeChanged_msg

Ardrone3PilotingStateAltitudeChanged
####################################################################################
Drones altitude changed.\n The altitude reported is the altitude above the take off point.\n To get the altitude above sea level, see [PositionChanged](#1-4-4).

- Parameter: ``~states/enable_pilotingstate_altitudechanged``
- Topic: ``states/ardrone3/PilotingState/AltitudeChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateAltitudeChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateAltitudeChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateAltitudeChanged.msg
  :name: Ardrone3PilotingStateAltitudeChanged_msg

Ardrone3PilotingStateGpsLocationChanged
####################################################################################
Drones location changed.\n This event is meant to replace [PositionChanged](#1-4-4).

- Parameter: ``~states/enable_pilotingstate_gpslocationchanged``
- Topic: ``states/ardrone3/PilotingState/GpsLocationChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateGpsLocationChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateGpsLocationChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateGpsLocationChanged.msg
  :name: Ardrone3PilotingStateGpsLocationChanged_msg

Ardrone3PilotingStateLandingStateChanged
####################################################################################
Landing state.\n Only available for fixed wings (which have two landing modes).

- Parameter: ``~states/enable_pilotingstate_landingstatechanged``
- Topic: ``states/ardrone3/PilotingState/LandingStateChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateLandingStateChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateLandingStateChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateLandingStateChanged.msg
  :name: Ardrone3PilotingStateLandingStateChanged_msg

Ardrone3PilotingStateAirSpeedChanged
####################################################################################
Drones air speed changed\n Expressed in the drones referential.

- Parameter: ``~states/enable_pilotingstate_airspeedchanged``
- Topic: ``states/ardrone3/PilotingState/AirSpeedChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStateAirSpeedChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStateAirSpeedChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStateAirSpeedChanged.msg
  :name: Ardrone3PilotingStateAirSpeedChanged_msg

Ardrone3PilotingStatemoveToChanged
####################################################################################
The drone moves or moved to a given location.

- Parameter: ``~states/enable_pilotingstate_movetochanged``
- Topic: ``states/ardrone3/PilotingState/moveToChanged``
- Message type: ``bebop_msgs::Ardrone3PilotingStatemoveToChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PilotingStatemoveToChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PilotingStatemoveToChanged.msg
  :name: Ardrone3PilotingStatemoveToChanged_msg

Ardrone3NetworkStateWifiScanListChanged
####################################################################################
Wifi scan results.\n Please note that the list is not complete until you receive the event [WifiScanEnded](#1-14-1).

- Parameter: ``~states/enable_networkstate_wifiscanlistchanged``
- Topic: ``states/ardrone3/NetworkState/WifiScanListChanged``
- Message type: ``bebop_msgs::Ardrone3NetworkStateWifiScanListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3NetworkStateWifiScanListChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3NetworkStateWifiScanListChanged.msg
  :name: Ardrone3NetworkStateWifiScanListChanged_msg

Ardrone3NetworkStateAllWifiScanChanged
####################################################################################
Wifi scan ended.\n When receiving this event, the list of [WifiScanResults](#1-14-0) is complete.

- Parameter: ``~states/enable_networkstate_allwifiscanchanged``
- Topic: ``states/ardrone3/NetworkState/AllWifiScanChanged``
- Message type: ``bebop_msgs::Ardrone3NetworkStateAllWifiScanChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3NetworkStateAllWifiScanChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3NetworkStateAllWifiScanChanged.msg
  :name: Ardrone3NetworkStateAllWifiScanChanged_msg

Ardrone3NetworkStateWifiAuthChannelListChanged
####################################################################################
Available wifi channels.\n Please note that the list is not complete until you receive the event [AvailableWifiChannelsCompleted](#1-14-3).

- Parameter: ``~states/enable_networkstate_wifiauthchannellistchanged``
- Topic: ``states/ardrone3/NetworkState/WifiAuthChannelListChanged``
- Message type: ``bebop_msgs::Ardrone3NetworkStateWifiAuthChannelListChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3NetworkStateWifiAuthChannelListChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3NetworkStateWifiAuthChannelListChanged.msg
  :name: Ardrone3NetworkStateWifiAuthChannelListChanged_msg

Ardrone3NetworkStateAllWifiAuthChannelChanged
####################################################################################
Available wifi channels completed.\n When receiving this event, the list of [AvailableWifiChannels](#1-14-2) is complete.

- Parameter: ``~states/enable_networkstate_allwifiauthchannelchanged``
- Topic: ``states/ardrone3/NetworkState/AllWifiAuthChannelChanged``
- Message type: ``bebop_msgs::Ardrone3NetworkStateAllWifiAuthChannelChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3NetworkStateAllWifiAuthChannelChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3NetworkStateAllWifiAuthChannelChanged.msg
  :name: Ardrone3NetworkStateAllWifiAuthChannelChanged_msg

Ardrone3MediaStreamingStateVideoEnableChanged
####################################################################################
Video stream state.

- Parameter: ``~states/enable_mediastreamingstate_videoenablechanged``
- Topic: ``states/ardrone3/MediaStreamingState/VideoEnableChanged``
- Message type: ``bebop_msgs::Ardrone3MediaStreamingStateVideoEnableChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaStreamingStateVideoEnableChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaStreamingStateVideoEnableChanged.msg
  :name: Ardrone3MediaStreamingStateVideoEnableChanged_msg

Ardrone3MediaStreamingStateVideoStreamModeChanged
####################################################################################


- Parameter: ``~states/enable_mediastreamingstate_videostreammodechanged``
- Topic: ``states/ardrone3/MediaStreamingState/VideoStreamModeChanged``
- Message type: ``bebop_msgs::Ardrone3MediaStreamingStateVideoStreamModeChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3MediaStreamingStateVideoStreamModeChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3MediaStreamingStateVideoStreamModeChanged.msg
  :name: Ardrone3MediaStreamingStateVideoStreamModeChanged_msg

Ardrone3CameraStateOrientation
####################################################################################
Camera orientation.

- Parameter: ``~states/enable_camerastate_orientation``
- Topic: ``states/ardrone3/CameraState/Orientation``
- Message type: ``bebop_msgs::Ardrone3CameraStateOrientation``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3CameraStateOrientation.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3CameraStateOrientation.msg
  :name: Ardrone3CameraStateOrientation_msg

Ardrone3CameraStatedefaultCameraOrientation
####################################################################################
Orientation of the center of the camera.\n This is the value to send when you want to center the camera.

- Parameter: ``~states/enable_camerastate_defaultcameraorientation``
- Topic: ``states/ardrone3/CameraState/defaultCameraOrientation``
- Message type: ``bebop_msgs::Ardrone3CameraStatedefaultCameraOrientation``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3CameraStatedefaultCameraOrientation.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3CameraStatedefaultCameraOrientation.msg
  :name: Ardrone3CameraStatedefaultCameraOrientation_msg

Ardrone3CameraStateOrientationV2
####################################################################################
Camera orientation with float arguments.

- Parameter: ``~states/enable_camerastate_orientationv2``
- Topic: ``states/ardrone3/CameraState/OrientationV2``
- Message type: ``bebop_msgs::Ardrone3CameraStateOrientationV2``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3CameraStateOrientationV2.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3CameraStateOrientationV2.msg
  :name: Ardrone3CameraStateOrientationV2_msg

Ardrone3CameraStatedefaultCameraOrientationV2
####################################################################################
Orientation of the center of the camera.\n This is the value to send when you want to center the camera.

- Parameter: ``~states/enable_camerastate_defaultcameraorientationv2``
- Topic: ``states/ardrone3/CameraState/defaultCameraOrientationV2``
- Message type: ``bebop_msgs::Ardrone3CameraStatedefaultCameraOrientationV2``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3CameraStatedefaultCameraOrientationV2.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3CameraStatedefaultCameraOrientationV2.msg
  :name: Ardrone3CameraStatedefaultCameraOrientationV2_msg

Ardrone3CameraStateVelocityRange
####################################################################################
Camera Orientation velocity limits.

- Parameter: ``~states/enable_camerastate_velocityrange``
- Topic: ``states/ardrone3/CameraState/VelocityRange``
- Message type: ``bebop_msgs::Ardrone3CameraStateVelocityRange``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3CameraStateVelocityRange.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3CameraStateVelocityRange.msg
  :name: Ardrone3CameraStateVelocityRange_msg

Ardrone3AntiflickeringStateelectricFrequencyChanged
####################################################################################
Electric frequency.\n This piece of information is used for the antiflickering when the [AntiflickeringMode](#1-30-1) is set to *auto*.

- Parameter: ``~states/enable_antiflickeringstate_electricfrequencychanged``
- Topic: ``states/ardrone3/AntiflickeringState/electricFrequencyChanged``
- Message type: ``bebop_msgs::Ardrone3AntiflickeringStateelectricFrequencyChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3AntiflickeringStateelectricFrequencyChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3AntiflickeringStateelectricFrequencyChanged.msg
  :name: Ardrone3AntiflickeringStateelectricFrequencyChanged_msg

Ardrone3AntiflickeringStatemodeChanged
####################################################################################
Antiflickering mode.

- Parameter: ``~states/enable_antiflickeringstate_modechanged``
- Topic: ``states/ardrone3/AntiflickeringState/modeChanged``
- Message type: ``bebop_msgs::Ardrone3AntiflickeringStatemodeChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3AntiflickeringStatemodeChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3AntiflickeringStatemodeChanged.msg
  :name: Ardrone3AntiflickeringStatemodeChanged_msg

Ardrone3GPSStateNumberOfSatelliteChanged
####################################################################################
Number of GPS satellites.

- Parameter: ``~states/enable_gpsstate_numberofsatellitechanged``
- Topic: ``states/ardrone3/GPSState/NumberOfSatelliteChanged``
- Message type: ``bebop_msgs::Ardrone3GPSStateNumberOfSatelliteChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3GPSStateNumberOfSatelliteChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3GPSStateNumberOfSatelliteChanged.msg
  :name: Ardrone3GPSStateNumberOfSatelliteChanged_msg

Ardrone3GPSStateHomeTypeAvailabilityChanged
####################################################################################
Home type availability.

- Parameter: ``~states/enable_gpsstate_hometypeavailabilitychanged``
- Topic: ``states/ardrone3/GPSState/HomeTypeAvailabilityChanged``
- Message type: ``bebop_msgs::Ardrone3GPSStateHomeTypeAvailabilityChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3GPSStateHomeTypeAvailabilityChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3GPSStateHomeTypeAvailabilityChanged.msg
  :name: Ardrone3GPSStateHomeTypeAvailabilityChanged_msg

Ardrone3GPSStateHomeTypeChosenChanged
####################################################################################
Home type.\n This choice is made by the drone, according to the [PreferredHomeType](#1-24-4) and the [HomeTypeAvailability](#1-31-1). The drone will choose the type matching with the user preference only if this type is available. If not, it will chose a type in this order:\n FOLLOWEE ; TAKEOFF ; PILOT ; FIRST_FIX

- Parameter: ``~states/enable_gpsstate_hometypechosenchanged``
- Topic: ``states/ardrone3/GPSState/HomeTypeChosenChanged``
- Message type: ``bebop_msgs::Ardrone3GPSStateHomeTypeChosenChanged``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3GPSStateHomeTypeChosenChanged.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3GPSStateHomeTypeChosenChanged.msg
  :name: Ardrone3GPSStateHomeTypeChosenChanged_msg

Ardrone3PROStateFeatures
####################################################################################
Pro features.

- Parameter: ``~states/enable_prostate_features``
- Topic: ``states/ardrone3/PROState/Features``
- Message type: ``bebop_msgs::Ardrone3PROStateFeatures``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3PROStateFeatures.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3PROStateFeatures.msg
  :name: Ardrone3PROStateFeatures_msg

Ardrone3AccessoryStateConnectedAccessories
####################################################################################
List of all connected accessories. This event presents the list of all connected accessories. To actually use the component, use the component dedicated feature.

- Parameter: ``~states/enable_accessorystate_connectedaccessories``
- Topic: ``states/ardrone3/AccessoryState/ConnectedAccessories``
- Message type: ``bebop_msgs::Ardrone3AccessoryStateConnectedAccessories``

.. literalinclude::
  ../../bebop_msgs/msg/autogenerated/Ardrone3AccessoryStateConnectedAccessories.msg
  :lines: 8-
  :language: python
  :caption: Ardrone3AccessoryStateConnectedAccessories.msg
  :name: Ardrone3AccessoryStateConnectedAccessories_msg

