/*
 * Ardrone3_setting_callback_includes.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
 * Date: 2015-09-02
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#ifdef DEFINE_SHARED_PTRS
// Define all callback wrappers
boost::shared_ptr<cb::PilotingSettingsMaxAltitude> ardrone3_pilotingsettings_maxaltitude_ptr;
boost::shared_ptr<cb::PilotingSettingsMaxTilt> ardrone3_pilotingsettings_maxtilt_ptr;
boost::shared_ptr<cb::PilotingSettingsAbsolutControl> ardrone3_pilotingsettings_absolutcontrol_ptr;
boost::shared_ptr<cb::PilotingSettingsMaxDistance> ardrone3_pilotingsettings_maxdistance_ptr;
boost::shared_ptr<cb::PilotingSettingsNoFlyOverMaxDistance> ardrone3_pilotingsettings_noflyovermaxdistance_ptr;
boost::shared_ptr<cb::SpeedSettingsMaxVerticalSpeed> ardrone3_speedsettings_maxverticalspeed_ptr;
boost::shared_ptr<cb::SpeedSettingsMaxRotationSpeed> ardrone3_speedsettings_maxrotationspeed_ptr;
boost::shared_ptr<cb::SpeedSettingsHullProtection> ardrone3_speedsettings_hullprotection_ptr;
boost::shared_ptr<cb::SpeedSettingsOutdoor> ardrone3_speedsettings_outdoor_ptr;
boost::shared_ptr<cb::NetworkSettingsWifiSelection> ardrone3_networksettings_wifiselection_ptr;
boost::shared_ptr<cb::GPSSettingsHomeType> ardrone3_gpssettings_hometype_ptr;
boost::shared_ptr<cb::GPSSettingsReturnHomeDelay> ardrone3_gpssettings_returnhomedelay_ptr;
#endif  // DEFINE_SHARED_PTRS

#ifdef UPDTAE_CALLBACK_MAP
// Instantiate state callback wrappers
ardrone3_pilotingsettings_maxaltitude_ptr.reset(new cb::PilotingSettingsMaxAltitude());
ardrone3_pilotingsettings_maxtilt_ptr.reset(new cb::PilotingSettingsMaxTilt());
ardrone3_pilotingsettings_absolutcontrol_ptr.reset(new cb::PilotingSettingsAbsolutControl());
ardrone3_pilotingsettings_maxdistance_ptr.reset(new cb::PilotingSettingsMaxDistance());
ardrone3_pilotingsettings_noflyovermaxdistance_ptr.reset(new cb::PilotingSettingsNoFlyOverMaxDistance());
ardrone3_speedsettings_maxverticalspeed_ptr.reset(new cb::SpeedSettingsMaxVerticalSpeed());
ardrone3_speedsettings_maxrotationspeed_ptr.reset(new cb::SpeedSettingsMaxRotationSpeed());
ardrone3_speedsettings_hullprotection_ptr.reset(new cb::SpeedSettingsHullProtection());
ardrone3_speedsettings_outdoor_ptr.reset(new cb::SpeedSettingsOutdoor());
ardrone3_networksettings_wifiselection_ptr.reset(new cb::NetworkSettingsWifiSelection());
ardrone3_gpssettings_hometype_ptr.reset(new cb::GPSSettingsHomeType());
ardrone3_gpssettings_returnhomedelay_ptr.reset(new cb::GPSSettingsReturnHomeDelay());

// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_maxaltitude_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_maxaltitude_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_maxtilt_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_maxtilt_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_absolutcontrol_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_absolutcontrol_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_maxdistance_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_maxdistance_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_pilotingsettings_noflyovermaxdistance_ptr->GetCommandKey(),
                      ardrone3_pilotingsettings_noflyovermaxdistance_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_maxverticalspeed_ptr->GetCommandKey(),
                      ardrone3_speedsettings_maxverticalspeed_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_maxrotationspeed_ptr->GetCommandKey(),
                      ardrone3_speedsettings_maxrotationspeed_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_hullprotection_ptr->GetCommandKey(),
                      ardrone3_speedsettings_hullprotection_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_speedsettings_outdoor_ptr->GetCommandKey(),
                      ardrone3_speedsettings_outdoor_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_networksettings_wifiselection_ptr->GetCommandKey(),
                      ardrone3_networksettings_wifiselection_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpssettings_hometype_ptr->GetCommandKey(),
                      ardrone3_gpssettings_hometype_ptr));
// Add all wrappers to the callback map
callback_map_.insert(std::pair<eARCONTROLLER_DICTIONARY_KEY, boost::shared_ptr<cb::AbstractCommand> >(
                      ardrone3_gpssettings_returnhomedelay_ptr->GetCommandKey(),
                      ardrone3_gpssettings_returnhomedelay_ptr));
#endif  // UPDTAE_CALLBACK_MAP