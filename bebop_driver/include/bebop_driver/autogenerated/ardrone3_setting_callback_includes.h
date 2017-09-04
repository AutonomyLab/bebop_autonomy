/**
Software License Agreement (BSD)

\file      Ardrone3_setting_callback_includes.h
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

 * Ardrone3_setting_callback_includes.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/ab28dab91845cd36c4d7002b55f70805deaff3c8/xml/ardrone3.xml
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#ifdef FORWARD_DECLARATIONS
namespace cb
{
  class PilotingSettingsMaxAltitude;
  class PilotingSettingsMaxTilt;
  class PilotingSettingsAbsolutControl;
  class PilotingSettingsMaxDistance;
  class PilotingSettingsNoFlyOverMaxDistance;
  class PilotingSettingsBankedTurn;
  class PilotingSettingsMinAltitude;
  class PilotingSettingsCirclingDirection;
  class PilotingSettingsCirclingRadius;
  class PilotingSettingsCirclingAltitude;
  class PilotingSettingsPitchMode;
  class SpeedSettingsMaxVerticalSpeed;
  class SpeedSettingsMaxRotationSpeed;
  class SpeedSettingsHullProtection;
  class SpeedSettingsOutdoor;
  class SpeedSettingsMaxPitchRollRotationSpeed;
  class NetworkSettingsWifiSelection;
  class PictureSettingsVideoStabilizationMode;
  class PictureSettingsVideoRecordingMode;
  class PictureSettingsVideoFramerate;
  class PictureSettingsVideoResolutions;
  class GPSSettingsHomeType;
  class GPSSettingsReturnHomeDelay;
}  // namespace cb
#endif  // FORWARD_DECLARATIONS

#ifdef DEFINE_SHARED_PTRS
// Define all callback wrappers
boost::shared_ptr<cb::PilotingSettingsMaxAltitude> ardrone3_pilotingsettings_maxaltitude_ptr;
boost::shared_ptr<cb::PilotingSettingsMaxTilt> ardrone3_pilotingsettings_maxtilt_ptr;
boost::shared_ptr<cb::PilotingSettingsAbsolutControl> ardrone3_pilotingsettings_absolutcontrol_ptr;
boost::shared_ptr<cb::PilotingSettingsMaxDistance> ardrone3_pilotingsettings_maxdistance_ptr;
boost::shared_ptr<cb::PilotingSettingsNoFlyOverMaxDistance> ardrone3_pilotingsettings_noflyovermaxdistance_ptr;
boost::shared_ptr<cb::PilotingSettingsBankedTurn> ardrone3_pilotingsettings_bankedturn_ptr;
boost::shared_ptr<cb::PilotingSettingsMinAltitude> ardrone3_pilotingsettings_minaltitude_ptr;
boost::shared_ptr<cb::PilotingSettingsCirclingDirection> ardrone3_pilotingsettings_circlingdirection_ptr;
boost::shared_ptr<cb::PilotingSettingsCirclingRadius> ardrone3_pilotingsettings_circlingradius_ptr;
boost::shared_ptr<cb::PilotingSettingsCirclingAltitude> ardrone3_pilotingsettings_circlingaltitude_ptr;
boost::shared_ptr<cb::PilotingSettingsPitchMode> ardrone3_pilotingsettings_pitchmode_ptr;
boost::shared_ptr<cb::SpeedSettingsMaxVerticalSpeed> ardrone3_speedsettings_maxverticalspeed_ptr;
boost::shared_ptr<cb::SpeedSettingsMaxRotationSpeed> ardrone3_speedsettings_maxrotationspeed_ptr;
boost::shared_ptr<cb::SpeedSettingsHullProtection> ardrone3_speedsettings_hullprotection_ptr;
boost::shared_ptr<cb::SpeedSettingsOutdoor> ardrone3_speedsettings_outdoor_ptr;
boost::shared_ptr<cb::SpeedSettingsMaxPitchRollRotationSpeed> ardrone3_speedsettings_maxpitchrollrotationspeed_ptr;
boost::shared_ptr<cb::NetworkSettingsWifiSelection> ardrone3_networksettings_wifiselection_ptr;
boost::shared_ptr<cb::PictureSettingsVideoStabilizationMode> ardrone3_picturesettings_videostabilizationmode_ptr;
boost::shared_ptr<cb::PictureSettingsVideoRecordingMode> ardrone3_picturesettings_videorecordingmode_ptr;
boost::shared_ptr<cb::PictureSettingsVideoFramerate> ardrone3_picturesettings_videoframerate_ptr;
boost::shared_ptr<cb::PictureSettingsVideoResolutions> ardrone3_picturesettings_videoresolutions_ptr;
boost::shared_ptr<cb::GPSSettingsHomeType> ardrone3_gpssettings_hometype_ptr;
boost::shared_ptr<cb::GPSSettingsReturnHomeDelay> ardrone3_gpssettings_returnhomedelay_ptr;
#endif  // DEFINE_SHARED_PTRS

#ifdef UPDTAE_CALLBACK_MAP
// Instantiate state callback wrappers
ardrone3_pilotingsettings_maxaltitude_ptr.reset(new cb::PilotingSettingsMaxAltitude(priv_nh));
ardrone3_pilotingsettings_maxtilt_ptr.reset(new cb::PilotingSettingsMaxTilt(priv_nh));
ardrone3_pilotingsettings_absolutcontrol_ptr.reset(new cb::PilotingSettingsAbsolutControl(priv_nh));
ardrone3_pilotingsettings_maxdistance_ptr.reset(new cb::PilotingSettingsMaxDistance(priv_nh));
ardrone3_pilotingsettings_noflyovermaxdistance_ptr.reset(new cb::PilotingSettingsNoFlyOverMaxDistance(priv_nh));
ardrone3_pilotingsettings_bankedturn_ptr.reset(new cb::PilotingSettingsBankedTurn(priv_nh));
ardrone3_pilotingsettings_minaltitude_ptr.reset(new cb::PilotingSettingsMinAltitude(priv_nh));
ardrone3_pilotingsettings_circlingdirection_ptr.reset(new cb::PilotingSettingsCirclingDirection(priv_nh));
ardrone3_pilotingsettings_circlingradius_ptr.reset(new cb::PilotingSettingsCirclingRadius(priv_nh));
ardrone3_pilotingsettings_circlingaltitude_ptr.reset(new cb::PilotingSettingsCirclingAltitude(priv_nh));
ardrone3_pilotingsettings_pitchmode_ptr.reset(new cb::PilotingSettingsPitchMode(priv_nh));
ardrone3_speedsettings_maxverticalspeed_ptr.reset(new cb::SpeedSettingsMaxVerticalSpeed(priv_nh));
ardrone3_speedsettings_maxrotationspeed_ptr.reset(new cb::SpeedSettingsMaxRotationSpeed(priv_nh));
ardrone3_speedsettings_hullprotection_ptr.reset(new cb::SpeedSettingsHullProtection(priv_nh));
ardrone3_speedsettings_outdoor_ptr.reset(new cb::SpeedSettingsOutdoor(priv_nh));
ardrone3_speedsettings_maxpitchrollrotationspeed_ptr.reset(new cb::SpeedSettingsMaxPitchRollRotationSpeed(priv_nh));
ardrone3_networksettings_wifiselection_ptr.reset(new cb::NetworkSettingsWifiSelection(priv_nh));
ardrone3_picturesettings_videostabilizationmode_ptr.reset(new cb::PictureSettingsVideoStabilizationMode(priv_nh));
ardrone3_picturesettings_videorecordingmode_ptr.reset(new cb::PictureSettingsVideoRecordingMode(priv_nh));
ardrone3_picturesettings_videoframerate_ptr.reset(new cb::PictureSettingsVideoFramerate(priv_nh));
ardrone3_picturesettings_videoresolutions_ptr.reset(new cb::PictureSettingsVideoResolutions(priv_nh));
ardrone3_gpssettings_hometype_ptr.reset(new cb::GPSSettingsHomeType(priv_nh));
ardrone3_gpssettings_returnhomedelay_ptr.reset(new cb::GPSSettingsReturnHomeDelay(priv_nh));

// Add all wrappers to the callback map (AbstractCommand* part of each object)
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_maxaltitude_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_maxaltitude_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_maxtilt_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_maxtilt_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_absolutcontrol_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_absolutcontrol_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_maxdistance_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_maxdistance_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_noflyovermaxdistance_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_noflyovermaxdistance_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_bankedturn_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_bankedturn_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_minaltitude_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_minaltitude_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_circlingdirection_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_circlingdirection_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_circlingradius_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_circlingradius_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_circlingaltitude_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_circlingaltitude_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_pitchmode_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_pitchmode_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_maxverticalspeed_ptr->GetCommandKey(),
                      ardrone3_speedsettings_maxverticalspeed_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_maxrotationspeed_ptr->GetCommandKey(),
                      ardrone3_speedsettings_maxrotationspeed_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_hullprotection_ptr->GetCommandKey(),
                      ardrone3_speedsettings_hullprotection_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_outdoor_ptr->GetCommandKey(),
                      ardrone3_speedsettings_outdoor_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_maxpitchrollrotationspeed_ptr->GetCommandKey(),
                      ardrone3_speedsettings_maxpitchrollrotationspeed_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_networksettings_wifiselection_ptr->GetCommandKey(),
                      ardrone3_networksettings_wifiselection_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_picturesettings_videostabilizationmode_ptr->GetCommandKey(),
                      ardrone3_picturesettings_videostabilizationmode_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_picturesettings_videorecordingmode_ptr->GetCommandKey(),
                      ardrone3_picturesettings_videorecordingmode_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_picturesettings_videoframerate_ptr->GetCommandKey(),
                      ardrone3_picturesettings_videoframerate_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_picturesettings_videoresolutions_ptr->GetCommandKey(),
                      ardrone3_picturesettings_videoresolutions_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpssettings_hometype_ptr->GetCommandKey(),
                      ardrone3_gpssettings_hometype_ptr));
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpssettings_returnhomedelay_ptr->GetCommandKey(),
                      ardrone3_gpssettings_returnhomedelay_ptr));
#endif  // UPDTAE_CALLBACK_MAP
