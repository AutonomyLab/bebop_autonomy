****************************************************************************
bebop_autonomy - ROS Driver for Parrot Bebop Drone (quadrocopter) 1.0 & 2.0
****************************************************************************

*bebop_autonomy* is a :abbr:`ROS (Robot Operating System)` driver for `Parrot Bebop 1.0 <http://www.parrot.com/ca/products/bebop-drone/>`_ and 2.0 drones (quadrocopters), based on Parrot's official `ARDroneSDK3 <https://github.com/Parrot-Developers/arsdk_manifests>`_. This driver has been developed in `Autonomy Lab <http://autonomylab.org/>`_ of `Simon Fraser University <http://www.sfu.ca/>`_ by `Mani Monajjemi <http://mani.im>`_ and other contributers (:ref:`sec-contribs`).

[`Source Code <https://github.com/AutonomyLab/bebop_autonomy>`_] 
[`ROS wiki page <http://wiki.ros.org/bebop_autonomy>`_] 
[`Support <http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:bebop_autonomy/page:1/>`_] 
[`Bug Tracker <https://github.com/AutonomyLab/bebop_autonomy/issues>`_] 
[`Developer Forum <https://trello.com/b/C6rNl8Ux>`_]

.. _sec-roadmap:

Features and Roadmap
====================

.. csv-table::
  :header: "Feature", "Status", "Notes"

  Core piloting, Yes, ""
  H264 video decoding, Yes, "Enhancement: `#1 <https://github.com/AutonomyLab/bebop_autonomy/issues/1>`_"
  ROS Camera Interface, Yes, ""
  Nodelet implementation, Yes, ""
  Publish Bebop states as ROS topics, Yes, ""
  Dynamically reconfigurable Bebop settings, Yes, ":ref:`sec-dev-dyn`"
  Inline build of ARDroneSDK3, Yes, "Enhancement: `#2 <https://github.com/AutonomyLab/bebop_autonomy/issues/2>`_"
  Bebop In The Loop tests, Yes, ":ref:`sec-dev-test`"
  Joystick teleop demo, Yes, ""
  TF Publisher, Yes, "Since v0.5 (:ref:`sec-tf`)"
  Odometry Publisher, Yes, "Since v0.5 (:ref:`sec-odom`)"
  Provide ROS API for on-board picture/video recording, Yes, "Since v0.4.1 (:ref:`sec-snapshot`)"
  GPS Support, Yes, "Since v0.5 (:ref:`sec-gps`)"
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

