****************************************************************************
bebop_autonomy - ROS Driver for Parrot Bebop Drone (quadrocopter) 1.0 & 2.0
****************************************************************************

*bebop_autonomy* is a :abbr:`ROS (Robot Operating System)` driver for `Parrot Bebop 1.0 <http://www.parrot.com/ca/products/bebop-drone/>`_ and `2.0 <https://www.parrot.com/ca/drones/parrot-bebop-2>`_ drones (quadrocopters), based on Parrot's official `ARDroneSDK3 <http://developer.parrot.com/docs/SDK3/>`_. This driver has been developed in `Autonomy Lab <http://autonomylab.org/>`_ of `Simon Fraser University <http://www.sfu.ca/>`_ by `Mani Monajjemi <http://mani.im>`_ and other contributers (:ref:`sec-contribs`). This software is maintained by `Sepehr MohaimenianPour <http://sepehr.im/>`_ (AutonomyLab, Simon Fraser University), `Thomas Bamford <#>`_ (Dynamic Systems Lab, University of Toronto) and `Tobias Naegeli <https://ait.ethz.ch/people/naegelit/>`_ (Advanced Interactive Technologies Lab, ETH Zürich).

[`Source Code <https://github.com/AutonomyLab/bebop_autonomy>`_] 
[`ROS wiki page <http://wiki.ros.org/bebop_autonomy>`_] 
[`Support <http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:bebop_autonomy/page:1/>`_] 
[`Bug Tracker <https://github.com/AutonomyLab/bebop_autonomy/issues>`_] 

.. _sec-roadmap:

Features and Roadmap
====================

.. csv-table::
  :header: "Feature", "Status", "Notes"

  SDK Version,"3.12.6", "Since v0.7"
  Support for Parrot Bebop 1, Yes, "Tested up to Firmware 3.3"
  Support for Parrot Bebop 2, Yes, "Tested up to Firmware 4.0.6"
  Support for Parrot Disco FPV, No, "Not tested (help wanted)"
  Core piloting, Yes, ""
  H264 video decoding, Yes, "Enhancement: `#1 <https://github.com/AutonomyLab/bebop_autonomy/issues/1>`_"
  ROS Camera Interface, Yes, ""
  Nodelet implementation, Yes, ""
  Publish Bebop states as ROS topics, Yes, ""
  Dynamically reconfigurable Bebop settings, Yes, ":ref:`sec-dev-dyn`"
  Use `parrot_arsdk <https://github.com/AutonomyLab/parrot_arsdk>`_ instead of building ARSDK3 inline, Yes, "Since v0.6: `#75 <https://github.com/AutonomyLab/bebop_autonomy/issues/75>`_"
  Bebop In The Loop tests, Yes, ":ref:`sec-dev-test`"
  Joystick teleop demo, Yes, ":ref:`sec-pilot-teleop`"
  TF Publisher, Yes, "Since v0.5 (:ref:`sec-tf`)"
  Odometry Publisher, Yes, "Since v0.5 (:ref:`sec-odom`)"
  Provide ROS API for on-board picture/video recording, Yes, "Since v0.4.1 (:ref:`sec-snapshot`)"
  GPS Support, Yes, "Since v0.6 (:ref:`sec-gps`)"
  Support for 720p streaming, Yes, "Since v0.6"
  Mavlink Support, No, ""
  Binary Release, No, ""
  Support for Parrot Sky Controller, No, ""

Table of Contents
=================

.. toctree::
  :maxdepth: 2

  changelog
  installation
  running
  piloting
  reading
  configuration
  coordinates
  contribute
  FAQ
  dev
  license

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

