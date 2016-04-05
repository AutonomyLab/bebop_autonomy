************
Installation
************

Compiling From Source
=====================

Pre-requirements:

- ROS *Indigo* or *Jade* (Only tested on *Ubuntu*)
- Internet connection
- Ubuntu packages: ``build-esstential``, ``python-rosdep``, ``python-catkin-tools``
- Basic familiarity with building ROS packages

.. code-block:: bash

  $ sudo apt-get install build-essential python-rosdep python-catkin-tools

To compile from source, you need to clone the source code in a new or existing ``catkin`` workspace, use ``rosdep`` to install dependencies and finally compile the workspace using `catkin`. The following commands demonstrate this procedure in a newly created ``catkin`` workspace.

.. code-block:: bash

  # Create and initialize the workspace
  $ mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
  $ catkin init
  $ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
  # Update rosdep database and install dependencies
  $ rosdep update
  $ rosdep install --from-paths src -i
  # Build the workspace
  $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo

The first time build may take up to 15 minutes, since ARDroneSDK3's build script downloads and compiles ~20 packages from the Internet.

