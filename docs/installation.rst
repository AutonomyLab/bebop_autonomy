************
Installation
************

Compiling From Source
=====================

Pre-requirements:

- ROS *Indigo*, *Jade* or *Kinetic* (Only tested on *Ubuntu*)
- Ubuntu packages: ``build-esstential``, ``python-rosdep``, ``python-catkin-tools``
- Basic familiarity with building ROS packages

.. code-block:: bash

  $ sudo apt-get install build-essential python-rosdep python-catkin-tools

To compile from source, you need to clone the source code in a new or existing ``catkin`` workspace, use ``rosdep`` to install dependencies and finally compile the workspace using `catkin`. The following commands demonstrate this procedure in a newly created ``catkin`` workspace.

.. note:: Since version 0.6, `Parrot ARSDK <http://developer.parrot.com/docs/SDK3/>`_, the main underlying dependency of  *bebop_autonomy* is not build inline anymore. Instead, the slightly patched and catkinized version of it, called `parrot_arsdk <https://github.com/AutonomyLab/parrot_arsdk>`_, is fetched as a dependency during the ``rosdep install`` step below. This dramatically decreases the compile time of the package compared to previous versions (e.g. from ~15 minutes to less than a minute on a modern computer). If you are re-compiling from source, you need to clean your workspace first: ``$ catkin clean [-y]``.

.. code-block:: bash

  # Create and initialize the workspace
  $ mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
  $ catkin init
  $ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
  # Update rosdep database and install dependencies (including parrot_arsdk)
  $ rosdep update
  $ rosdep install --from-paths src -i
  # Build the workspace
  $ catkin build 

