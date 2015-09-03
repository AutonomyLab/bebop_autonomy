/*
 * ARDrone3_state_callback_includes.h
 * auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
 * Date: 2015-09-02
 * Do not modify this file by hand. Check scripts/meta folder for generator files.
 */

#ifdef DEFINE_SHARED_PTRS
// Define all callback wrappers
boost::shared_ptr<cb::Ardrone3MediaRecordStatePictureStateChanged> ardrone3_mediarecordstate_picturestatechanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStateVideoStateChanged> ardrone3_mediarecordstate_videostatechanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStatePictureStateChangedV2> ardrone3_mediarecordstate_picturestatechangedv2_ptr;
boost::shared_ptr<cb::Ardrone3MediaRecordStateVideoStateChangedV2> ardrone3_mediarecordstate_videostatechangedv2_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateFlatTrimChanged> ardrone3_pilotingstate_flattrimchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateFlyingStateChanged> ardrone3_pilotingstate_flyingstatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAlertStateChanged> ardrone3_pilotingstate_alertstatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateNavigateHomeStateChanged> ardrone3_pilotingstate_navigatehomestatechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStatePositionChanged> ardrone3_pilotingstate_positionchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateSpeedChanged> ardrone3_pilotingstate_speedchanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAttitudeChanged> ardrone3_pilotingstate_attitudechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAutoTakeOffModeChanged> ardrone3_pilotingstate_autotakeoffmodechanged_ptr;
boost::shared_ptr<cb::Ardrone3PilotingStateAltitudeChanged> ardrone3_pilotingstate_altitudechanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateWifiScanListChanged> ardrone3_networkstate_wifiscanlistchanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateAllWifiScanChanged> ardrone3_networkstate_allwifiscanchanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateWifiAuthChannelListChanged> ardrone3_networkstate_wifiauthchannellistchanged_ptr;
boost::shared_ptr<cb::Ardrone3NetworkStateAllWifiAuthChannelChanged> ardrone3_networkstate_allwifiauthchannelchanged_ptr;
boost::shared_ptr<cb::Ardrone3MediaStreamingStateVideoEnableChanged> ardrone3_mediastreamingstate_videoenablechanged_ptr;
boost::shared_ptr<cb::Ardrone3CameraStateOrientation> ardrone3_camerastate_orientation_ptr;
boost::shared_ptr<cb::Ardrone3AntiflickeringStateelectricFrequencyChanged> ardrone3_antiflickeringstate_electricfrequencychanged_ptr;
boost::shared_ptr<cb::Ardrone3AntiflickeringStatemodeChanged> ardrone3_antiflickeringstate_modechanged_ptr;
boost::shared_ptr<cb::Ardrone3GPSStateNumberOfSatelliteChanged> ardrone3_gpsstate_numberofsatellitechanged_ptr;
boost::shared_ptr<cb::Ardrone3GPSStateHomeTypeAvailabilityChanged> ardrone3_gpsstate_hometypeavailabilitychanged_ptr;
boost::shared_ptr<cb::Ardrone3GPSStateHomeTypeChosenChanged> ardrone3_gpsstate_hometypechosenchanged_ptr;
boost::shared_ptr<cb::Ardrone3PROStateFeatures> ardrone3_prostate_features_ptr;
#endif  // DEFINE_SHARED_PTRS

#ifdef UPDTAE_CALLBACK_MAP
// Instantiate state callback wrappers
ardrone3_mediarecordstate_picturestatechanged_ptr.reset(new cb::Ardrone3MediaRecordStatePictureStateChanged(nh, "states/ARDrone3/MediaRecordState/PictureStateChanged"));
ardrone3_mediarecordstate_videostatechanged_ptr.reset(new cb::Ardrone3MediaRecordStateVideoStateChanged(nh, "states/ARDrone3/MediaRecordState/VideoStateChanged"));
ardrone3_mediarecordstate_picturestatechangedv2_ptr.reset(new cb::Ardrone3MediaRecordStatePictureStateChangedV2(nh, "states/ARDrone3/MediaRecordState/PictureStateChangedV2"));
ardrone3_mediarecordstate_videostatechangedv2_ptr.reset(new cb::Ardrone3MediaRecordStateVideoStateChangedV2(nh, "states/ARDrone3/MediaRecordState/VideoStateChangedV2"));
ardrone3_pilotingstate_flattrimchanged_ptr.reset(new cb::Ardrone3PilotingStateFlatTrimChanged(nh, "states/ARDrone3/PilotingState/FlatTrimChanged"));
ardrone3_pilotingstate_flyingstatechanged_ptr.reset(new cb::Ardrone3PilotingStateFlyingStateChanged(nh, "states/ARDrone3/PilotingState/FlyingStateChanged"));
ardrone3_pilotingstate_alertstatechanged_ptr.reset(new cb::Ardrone3PilotingStateAlertStateChanged(nh, "states/ARDrone3/PilotingState/AlertStateChanged"));
ardrone3_pilotingstate_navigatehomestatechanged_ptr.reset(new cb::Ardrone3PilotingStateNavigateHomeStateChanged(nh, "states/ARDrone3/PilotingState/NavigateHomeStateChanged"));
ardrone3_pilotingstate_positionchanged_ptr.reset(new cb::Ardrone3PilotingStatePositionChanged(nh, "states/ARDrone3/PilotingState/PositionChanged"));
ardrone3_pilotingstate_speedchanged_ptr.reset(new cb::Ardrone3PilotingStateSpeedChanged(nh, "states/ARDrone3/PilotingState/SpeedChanged"));
ardrone3_pilotingstate_attitudechanged_ptr.reset(new cb::Ardrone3PilotingStateAttitudeChanged(nh, "states/ARDrone3/PilotingState/AttitudeChanged"));
ardrone3_pilotingstate_autotakeoffmodechanged_ptr.reset(new cb::Ardrone3PilotingStateAutoTakeOffModeChanged(nh, "states/ARDrone3/PilotingState/AutoTakeOffModeChanged"));
ardrone3_pilotingstate_altitudechanged_ptr.reset(new cb::Ardrone3PilotingStateAltitudeChanged(nh, "states/ARDrone3/PilotingState/AltitudeChanged"));
ardrone3_networkstate_wifiscanlistchanged_ptr.reset(new cb::Ardrone3NetworkStateWifiScanListChanged(nh, "states/ARDrone3/NetworkState/WifiScanListChanged"));
ardrone3_networkstate_allwifiscanchanged_ptr.reset(new cb::Ardrone3NetworkStateAllWifiScanChanged(nh, "states/ARDrone3/NetworkState/AllWifiScanChanged"));
ardrone3_networkstate_wifiauthchannellistchanged_ptr.reset(new cb::Ardrone3NetworkStateWifiAuthChannelListChanged(nh, "states/ARDrone3/NetworkState/WifiAuthChannelListChanged"));
ardrone3_networkstate_allwifiauthchannelchanged_ptr.reset(new cb::Ardrone3NetworkStateAllWifiAuthChannelChanged(nh, "states/ARDrone3/NetworkState/AllWifiAuthChannelChanged"));
ardrone3_mediastreamingstate_videoenablechanged_ptr.reset(new cb::Ardrone3MediaStreamingStateVideoEnableChanged(nh, "states/ARDrone3/MediaStreamingState/VideoEnableChanged"));
ardrone3_camerastate_orientation_ptr.reset(new cb::Ardrone3CameraStateOrientation(nh, "states/ARDrone3/CameraState/Orientation"));
ardrone3_antiflickeringstate_electricfrequencychanged_ptr.reset(new cb::Ardrone3AntiflickeringStateelectricFrequencyChanged(nh, "states/ARDrone3/AntiflickeringState/electricFrequencyChanged"));
ardrone3_antiflickeringstate_modechanged_ptr.reset(new cb::Ardrone3AntiflickeringStatemodeChanged(nh, "states/ARDrone3/AntiflickeringState/modeChanged"));
ardrone3_gpsstate_numberofsatellitechanged_ptr.reset(new cb::Ardrone3GPSStateNumberOfSatelliteChanged(nh, "states/ARDrone3/GPSState/NumberOfSatelliteChanged"));
ardrone3_gpsstate_hometypeavailabilitychanged_ptr.reset(new cb::Ardrone3GPSStateHomeTypeAvailabilityChanged(nh, "states/ARDrone3/GPSState/HomeTypeAvailabilityChanged"));
ardrone3_gpsstate_hometypechosenchanged_ptr.reset(new cb::Ardrone3GPSStateHomeTypeChosenChanged(nh, "states/ARDrone3/GPSState/HomeTypeChosenChanged"));
ardrone3_prostate_features_ptr.reset(new cb::Ardrone3PROStateFeatures(nh, "states/ARDrone3/PROState/Features"));

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
                      ardrone3_camerastate_orientation_ptr->GetCommandKey(),
                      ardrone3_camerastate_orientation_ptr));
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
#endif  // UPDTAE_CALLBACK_MAP