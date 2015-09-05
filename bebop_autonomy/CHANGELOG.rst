^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bebop_autonomy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add support for downloading and building ARDroneSDK3 during the build process
* Add flattrim, flip and navigatehome interfaces
* Add forward declaration to classes where it is possible
* Major bug fixes and improvements
  - Dynamic Reconfigure: Convert all two state int_t values to enum
  - Fix the private nodehandle bugs in  State and Settings handlers
  - Fix the data flow of Settings between rosparam and dynamic reconfigure
  and bebop
  - Fix SDK enum types in C (I32 instead of U8)
  - Add Start/Stop streaming to Bebop interface class
* Add bebop_nodelet launch with image_view
* Organized DynR configs into groups
  + Moved the autogeneration report to a seperated file
  + build speed improvements
* Dynamically reconfigurable Bebop settings
* Add support to enable publishing of a specific State
* Add support to propogate states from bebop to ROS
* Auto-generated .msg and .h files based on libARCommands XML files
* New threading model for data retreival and publishing
  - Nodelet now manages its own thread to receive frames from Bebop
  - GetFrame() function abstracts all sync to access the rgb frame
  - All subscribers send commands to the Bebop in their callbacks
* Integreate ARSAL logs into ROS_LOG
  - Fix sync issues between frame grabber and publisher
* Improve video decode/publish pipeline
  - Adopt frame decoding from official examples
  - Thread safe access to raw frame ptr
  - Synchronised frame decoding and publishing
* Proof of concept ROS driver for bebop drone
* Contributors: Mani Monajjemi
