******************************************************************
bebop_autonomy - ROS Driver for Parrot Bebbop Drone (quadrocopter)
******************************************************************

*bebop_autonomy* is a :abbr:`ROS (Robot Operating System)` driver for `Parrot Bebop drone <http://www.parrot.com/ca/products/bebop-drone/>`_ (quadrocopter), based on Parrot's official `ARDroneSDK3 <https://github.com/Parrot-Developers/ARSDKBuildUtils>`_. This driver has been developed in `Autonomy Lab <http://autonomylab.org/>`_ of `Simon Fraser University <http://www.sfu.ca/>`_ by `Mani Monajjemi <http://mani.im>`_.

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
  TF Publisher, No (Planned), "`#3 <https://github.com/AutonomyLab/bebop_autonomy/issues/3>`_"
  Odometry Publisher, No (Planned), "`#4 <https://github.com/AutonomyLab/bebop_autonomy/issues/4>`_"
  Provide ROS API for on-board picture/video recording, No (Planned), "`#5 <https://github.com/AutonomyLab/bebop_autonomy/issues/5>`_"
  GPS Support, Partial, "Not fully tested"
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

