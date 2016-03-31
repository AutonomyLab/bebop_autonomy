*************************
Sending Commands to Bebop
*************************

.. note:: ``bebop_tools`` package comes with a launch file for tele-operating Bebop with a joystick using ROS `joy_teleop <http://wiki.ros.org/joy_teleop>`_ package. The configuration file (key-action map) is written for `Logitech F710 controller <http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad>`_ and is located in ``bebop_tools/config`` folder. Adapting the file to your own controller is straightforward. To teleop Bebop while the driver is running execute ``roslaunch bebop_tools joy_teleop.launch``.

Takeoff
=======

Publish a message of type ``std_msgs/Empty`` to ``takeoff`` topic.

.. code-block:: bash

  $ rostopic pub --once std_msgs/Empty [namespace]/takeoff

Land
====

Publish a message of type ``std_msgs/Empty`` to ``land`` topic.

.. code-block:: bash

  $ rostopic pub --once std_msgs/Empty [namespace]/land

Emergency
=========

Publish a message of type ``std_msgs/Empty`` to ``reset`` topic.

.. code-block:: bash

  $ rostopic pub --once std_msgs/Empty [namespace]/reset

Piloting
========

To move Bebop around, publish messages of type `geometry_msgs/Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_ to `cmd_vel` topic while Bebop is flying. The effect of each field of the message on Bebop's movement is listed below:

.. code-block:: text

  linear.x  (+)      Translate forward
            (-)      Translate backward
  linear.y  (+)      Translate to left
            (-)      Translate to right
  linear.z  (+)      Ascend
            (-)      Descend
  angular.z (+)      Rotate counter clockwise
            (-)      Rotate clockwise

Acceptable range for all fields are ``[-1..1]``. The drone executes the last received command as long as the driver is running. This command is reset to when Takeoff_, Land_ or Emergency_ command is received. To make Bebop hover and maintain its current position, you need to publish a message with all fields set to zero to ``cmd_vel``.

.. note:: Since version 0.4, sign of ``angular.z`` has been negated to conform with :ref:`sec-coords` and `ardrone_autonomy <http://wiki.ros.org/ardrone_autonomy>`_.

Moving the Virtual Camera
=========================

To move Bebop's virtual camera, publish a message of type `geometry_msgs/Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_ to `camera_control` topic. ``angular.y`` and ``angular.z`` fields of this message set **absolute** tilt and pan of the camera in **degrees** respectively. The field of view of this virtual camera (based on our measurements) is ~80 (horizontal) and ~50 (vertical) degrees.

.. warning:: The API for this command is not stable. We plan to use ``JointState`` message in future versions.

.. code-block:: text

  angular.y (+)      tilt down
            (-)      tilt up
  angular.z (+)      pan left
            (-)      pan right

GPS Navigation
==============

.. warning:: Not fully integrated/tested yet.

Flat Trim
=========

.. error:: Test fails, probably not working.

Publish a message of type ``std_msgs/Empty`` to ``flattrim`` topic.

.. code-block:: bash

  $ rostopic pub --once std_msgs/Empty [namespace]/flattrim

Flight Animations
=================

.. warning:: Be extra cautious when performing any flight animations, specially in indoor environments.

Bebop can perform four different types of flight animation (flipping). To perform an animation, publish a message of type ``std_msgs/UInt8`` to `flip` topic while drone is flying. The ``data`` field determines the requested animation type.


.. code-block:: text

  0       Flip Forward
  1       Flip Backward
  2       Flip Right
  3       Flip Left

.. _sec-snapshot:

Take on-board Snapshot
======================

.. versionadded:: 0.4.1

To take a high resolution on-board snapshot, publish a ``std_msgs/Empty`` message on ``snapshot`` topic. The resulting snapshot is stored on the internal storage of the Bebop. The quality and type of this image is not configurable using the ROS driver. You can use the official FreeFlight3 app to configure your Bebop prior to flying. To access the on-board media, either connect your Bebop over USB to a computer, or use a FTP client to connect to your Bebop using the following settings:

* Default IP: ``192.168.42.11``
* Port: ``21``
* Path: ``internal_000/Bebop_Drone/media``
* Username: ``anonymous``
* Password: *<no password>*

Toggle on-board Video Recording
===============================

.. versionadded:: 0.4.1

To start or stop on-board high-resolution video recording, publish a ``std_msgs/Bool`` message on the ``record`` topic. The value of ``true`` starts the recording while the value of ``false`` stops it. Please refer to the previous section for information on how to access the on-board recorded media.
