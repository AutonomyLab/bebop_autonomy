 .. Ardrone3_settings_param.rst
 .. auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/5898658a925245555153459ea4684aa87f220e07/Xml/ARDrone3_commands.xml
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

`speedsettings`_
  Speed Settings commands

  `SpeedSettingsMaxVerticalSpeedCurrent`_
   Current max vertical speed in m/s
  `SpeedSettingsMaxRotationSpeedCurrent`_
   Current max rotation speed in degree/s
  `SpeedSettingsHullProtectionPresent`_
   1 if present, 0 if not present
  `SpeedSettingsOutdoorOutdoor`_
   1 if outdoor flight, 0 if indoor flight

`networksettings`_
  Network settings commands

  `NetworkSettingsWifiSelectionType`_
   The type of wifi selection (auto, manual)
  `NetworkSettingsWifiSelectionBand`_
   The allowed band(s) : 2.4 Ghz, 5 Ghz, or all
  `NetworkSettingsWifiSelectionChannel`_
   The channel (not used in auto mode)

`settings`_
  Settings commands


`picturesettings`_
  Photo settings chosen by the user


`gpssettings`_
  GPS settings

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
* Details: Current max rotation speed in degree/s
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
settings
===========================================================
picturesettings
===========================================================
gpssettings
===========================================================
GPSSettingsHomeTypeType
-----------------------------------------------------------
* Parameter: ``~GPSSettingsHomeTypeType``
* Details: The type of the home position
* Type: ``int_t``

  * 0: The drone will try to return to the take off position
  * 1: The drone will try to return to the pilot position

GPSSettingsReturnHomeDelayDelay
-----------------------------------------------------------
* Parameter: ``~GPSSettingsReturnHomeDelayDelay``
* Details: Delay in second
* Type: ``int_t``
