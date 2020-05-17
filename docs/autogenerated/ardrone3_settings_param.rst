 .. Ardrone3_settings_param.rst
 .. auto-generated from https://raw.githubusercontent.com/bluecamel/arsdk-xml/53ed05daac60e1ff8391f09c8b6eb06c18a992b6/xml/ardrone3.xml
 .. Do not modify this file by hand. Check scripts/meta folder for generator files.

*****************************************************************************************
List of Ardrone3 Settings and Corresponding ROS Parameter
*****************************************************************************************

`pilotingsettings`_
  Piloting Settings commands

  `PilotingSettingsMaxAltitudeCurrent`_
   Current altitude max in m
  `PilotingSettingsMaxTiltCurrent`_
   Current tilt max in degree
  `PilotingSettingsAbsolutControlOn`_
   1 to enable, 0 to disable
  `PilotingSettingsMaxDistanceValue`_
   Current max distance in meter
  `PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover`_
   1 if the drone cant fly further than max distance, 0 if no limitation on the drone should be done
  `PilotingSettingsBankedTurnValue`_
   1 to enable, 0 to disable
  `PilotingSettingsMinAltitudeCurrent`_
   Current altitude min in m
  `PilotingSettingsCirclingDirectionValue`_
   The circling direction
  `PilotingSettingsCirclingRadiusValue`_
   The circling radius in meter
  `PilotingSettingsCirclingAltitudeValue`_
   The circling altitude in meter
  `PilotingSettingsPitchModeValue`_
   The Pitch mode

`speedsettings`_
  Speed Settings commands

  `SpeedSettingsMaxVerticalSpeedCurrent`_
   Current max vertical speed in m/s
  `SpeedSettingsMaxRotationSpeedCurrent`_
   Current max yaw rotation speed in degree/s
  `SpeedSettingsHullProtectionPresent`_
   1 if present, 0 if not present
  `SpeedSettingsOutdoorOutdoor`_
   1 if outdoor flight, 0 if indoor flight
  `SpeedSettingsMaxPitchRollRotationSpeedCurrent`_
   Current max pitch/roll rotation speed in degree/s

`networksettings`_
  Network settings commands

  `NetworkSettingsWifiSelectionType`_
   The type of wifi selection (auto, manual)
  `NetworkSettingsWifiSelectionBand`_
   The allowed band(s) : 2.4 Ghz, 5 Ghz, or all
  `NetworkSettingsWifiSelectionChannel`_
   The channel (not used in auto mode)

`picturesettings`_
  Photo settings chosen by the user

  `PictureSettingsPictureFormatSelectionType`_
   The type of photo format
  `PictureSettingsAutoWhiteBalanceSelectionType`_
   The type auto white balance
  `PictureSettingsExpositionSelectionValue`_
   Exposition value (bounds given by ExpositionChanged arg min and max, by default [-3:3])
  `PictureSettingsSaturationSelectionValue`_
   Saturation value (bounds given by SaturationChanged arg min and max, by default [-100:100])
  `PictureSettingsTimelapseSelectionEnabled`_
   1 if timelapse is enabled, 0 otherwise
  `PictureSettingsTimelapseSelectionInterval`_
   interval in seconds for taking pictures
  `PictureSettingsVideoStabilizationModeMode`_
   Video stabilization mode
  `PictureSettingsVideoRecordingModeMode`_
   Video recording mode
  `PictureSettingsVideoFramerateFramerate`_
   Video framerate
  `PictureSettingsVideoResolutionsType`_
   Video streaming and recording resolutions

`gpssettings`_
  GPS settings

  `GPSSettingsSetHomeLatitude`_
   Home latitude in decimal degrees
  `GPSSettingsSetHomeLongitude`_
   Home longitude in decimal degrees
  `GPSSettingsSetHomeAltitude`_
   Home altitude in meters
  `GPSSettingsHomeTypeType`_
   The type of the home position
  `GPSSettingsReturnHomeDelayDelay`_
   Delay in second


pilotingsettings
===========================================================
PilotingSettingsMaxAltitudeCurrent
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsMaxAltitudeCurrent``
* Details: Current altitude max in m
* Type: ``double_t``
PilotingSettingsMaxTiltCurrent
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsMaxTiltCurrent``
* Details: Current tilt max in degree
* Type: ``double_t``
PilotingSettingsAbsolutControlOn
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsAbsolutControlOn``
* Details: 1 to enable, 0 to disable
* Type: ``int_t``

  * 0: Disabled
  * 1: Enabled

PilotingSettingsMaxDistanceValue
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsMaxDistanceValue``
* Details: Current max distance in meter
* Type: ``double_t``
PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover``
* Details: 1 if the drone cant fly further than max distance, 0 if no limitation on the drone should be done
* Type: ``int_t``

  * 0: Disabled
  * 1: Enabled

PilotingSettingsBankedTurnValue
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsBankedTurnValue``
* Details: 1 to enable, 0 to disable
* Type: ``int_t``

  * 0: Disabled
  * 1: Enabled

PilotingSettingsMinAltitudeCurrent
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsMinAltitudeCurrent``
* Details: Current altitude min in m
* Type: ``double_t``
PilotingSettingsCirclingDirectionValue
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsCirclingDirectionValue``
* Details: The circling direction
* Type: ``int_t``

  * 0: Circling ClockWise
  * 1: Circling Counter ClockWise

PilotingSettingsCirclingRadiusValue
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsCirclingRadiusValue``
* Details: The circling radius in meter
* Type: ``int_t``
PilotingSettingsCirclingAltitudeValue
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsCirclingAltitudeValue``
* Details: The circling altitude in meter
* Type: ``int_t``
PilotingSettingsPitchModeValue
-----------------------------------------------------------
* Parameter: ``~PilotingSettingsPitchModeValue``
* Details: The Pitch mode
* Type: ``int_t``

  * 0: Positive pitch values will make the drone lower its nose. Negative pitch values will make the drone raise its nose.
  * 1: Pitch commands are inverted. Positive pitch values will make the drone raise its nose. Negative pitch values will make the drone lower its nose.

speedsettings
===========================================================
SpeedSettingsMaxVerticalSpeedCurrent
-----------------------------------------------------------
* Parameter: ``~SpeedSettingsMaxVerticalSpeedCurrent``
* Details: Current max vertical speed in m/s
* Type: ``double_t``
SpeedSettingsMaxRotationSpeedCurrent
-----------------------------------------------------------
* Parameter: ``~SpeedSettingsMaxRotationSpeedCurrent``
* Details: Current max yaw rotation speed in degree/s
* Type: ``double_t``
SpeedSettingsHullProtectionPresent
-----------------------------------------------------------
* Parameter: ``~SpeedSettingsHullProtectionPresent``
* Details: 1 if present, 0 if not present
* Type: ``int_t``

  * 0: Disabled
  * 1: Enabled

SpeedSettingsOutdoorOutdoor
-----------------------------------------------------------
* Parameter: ``~SpeedSettingsOutdoorOutdoor``
* Details: 1 if outdoor flight, 0 if indoor flight
* Type: ``int_t``

  * 0: Disabled
  * 1: Enabled

SpeedSettingsMaxPitchRollRotationSpeedCurrent
-----------------------------------------------------------
* Parameter: ``~SpeedSettingsMaxPitchRollRotationSpeedCurrent``
* Details: Current max pitch/roll rotation speed in degree/s
* Type: ``double_t``
networksettings
===========================================================
NetworkSettingsWifiSelectionType
-----------------------------------------------------------
* Parameter: ``~NetworkSettingsWifiSelectionType``
* Details: The type of wifi selection (auto, manual)
* Type: ``int_t``

  * 0: Auto selection
  * 1: Manual selection

NetworkSettingsWifiSelectionBand
-----------------------------------------------------------
* Parameter: ``~NetworkSettingsWifiSelectionBand``
* Details: The allowed band(s) : 2.4 Ghz, 5 Ghz, or all
* Type: ``int_t``

  * 0: 2.4 GHz band
  * 1: 5 GHz band
  * 2: Both 2.4 and 5 GHz bands

NetworkSettingsWifiSelectionChannel
-----------------------------------------------------------
* Parameter: ``~NetworkSettingsWifiSelectionChannel``
* Details: The channel (not used in auto mode)
* Type: ``int_t``
picturesettings
===========================================================
PictureSettingsPictureFormatSelectionType
-----------------------------------------------------------
* Parameter: ``~PictureSettingsPictureFormatSelectionType``
* Details: The type of photo format
* Type: ``int_t``

  * 0: Take raw image
  * 1: Take a 4:3 jpeg photo
  * 2: Take a 16:9 snapshot from camera
  * 3: Take jpeg fisheye image only

PictureSettingsAutoWhiteBalanceSelectionType
-----------------------------------------------------------
* Parameter: ``~PictureSettingsAutoWhiteBalanceSelectionType``
* Details: The type auto white balance
* Type: ``int_t``

  * 0: Auto guess of best white balance params
  * 1: Tungsten white balance
  * 2: Daylight white balance
  * 3: Cloudy white balance
  * 4: White balance for a flash

PictureSettingsExpositionSelectionValue
-----------------------------------------------------------
* Parameter: ``~PictureSettingsExpositionSelectionValue``
* Details: Exposition value (bounds given by ExpositionChanged arg min and max, by default [-3:3])
* Type: ``double_t``
PictureSettingsSaturationSelectionValue
-----------------------------------------------------------
* Parameter: ``~PictureSettingsSaturationSelectionValue``
* Details: Saturation value (bounds given by SaturationChanged arg min and max, by default [-100:100])
* Type: ``double_t``
PictureSettingsTimelapseSelectionEnabled
-----------------------------------------------------------
* Parameter: ``~PictureSettingsTimelapseSelectionEnabled``
* Details: 1 if timelapse is enabled, 0 otherwise
* Type: ``int_t``

  * 0: Disabled
  * 1: Enabled

PictureSettingsTimelapseSelectionInterval
-----------------------------------------------------------
* Parameter: ``~PictureSettingsTimelapseSelectionInterval``
* Details: interval in seconds for taking pictures
* Type: ``double_t``
PictureSettingsVideoStabilizationModeMode
-----------------------------------------------------------
* Parameter: ``~PictureSettingsVideoStabilizationModeMode``
* Details: Video stabilization mode
* Type: ``int_t``

  * 0: Video flat on roll and pitch
  * 1: Video flat on pitch only
  * 2: Video flat on roll only
  * 3: Video follows drone angles

PictureSettingsVideoRecordingModeMode
-----------------------------------------------------------
* Parameter: ``~PictureSettingsVideoRecordingModeMode``
* Details: Video recording mode
* Type: ``int_t``

  * 0: Maximize recording quality.
  * 1: Maximize recording time.

PictureSettingsVideoFramerateFramerate
-----------------------------------------------------------
* Parameter: ``~PictureSettingsVideoFramerateFramerate``
* Details: Video framerate
* Type: ``int_t``

  * 0: 23.976 frames per second.
  * 1: 25 frames per second.
  * 2: 29.97 frames per second.

PictureSettingsVideoResolutionsType
-----------------------------------------------------------
* Parameter: ``~PictureSettingsVideoResolutionsType``
* Details: Video streaming and recording resolutions
* Type: ``int_t``

  * 0: 1080p recording, 480p streaming.
  * 1: 720p recording, 720p streaming.

gpssettings
===========================================================
GPSSettingsSetHomeLatitude
-----------------------------------------------------------
* Parameter: ``~GPSSettingsSetHomeLatitude``
* Details: Home latitude in decimal degrees
* Type: ``double_t``
GPSSettingsSetHomeLongitude
-----------------------------------------------------------
* Parameter: ``~GPSSettingsSetHomeLongitude``
* Details: Home longitude in decimal degrees
* Type: ``double_t``
GPSSettingsSetHomeAltitude
-----------------------------------------------------------
* Parameter: ``~GPSSettingsSetHomeAltitude``
* Details: Home altitude in meters
* Type: ``double_t``
GPSSettingsHomeTypeType
-----------------------------------------------------------
* Parameter: ``~GPSSettingsHomeTypeType``
* Details: The type of the home position
* Type: ``int_t``

  * 0: The drone will try to return to the take off position
  * 1: The drone will try to return to the pilot position
  * 2: The drone will try to return to the target of the current (or last) follow me

GPSSettingsReturnHomeDelayDelay
-----------------------------------------------------------
* Parameter: ``~GPSSettingsReturnHomeDelayDelay``
* Details: Delay in second
* Type: ``int_t``
