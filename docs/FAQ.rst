**************************
Frequently Asked Questions
**************************

Is *bebop_autonomy* based on *ardrone_autonomy*?
================================================

No. `ardrone_autonomy <http://wiki.ros.org/ardrone_autonomy>`_ is based on Parrot's `legacy SDK <https://github.com/AutonomyLab/ardronelib>`_ for AR-Drone 1.0 and 2.0, while *bebop_autonomy* uses Parrot's new SDK for its third generation drones. Since these two SDKs and their underlying protocols are totally different and incompatible, we had to develop *bebop_autonomy* from scrath.

Is *bebop_autonomy* compatible with *ardrone_autonomy*?
=======================================================

Not completely.

- Topic names, types and coordinate frame conventions for core piloting tasks are identical, however there is no explicit namespacing (i.e. ``takeoff`` instead of ``ardrone/takeoff``)
- *bebop_autonomy* does not expose services for *Flight Animations* or *Flat Trim*; topics are used instead.
- Front camera video stream is published on ``image_raw`` topic only.
- Parameter names, types and effects are different.
- AR-Drone *Navdata* is replaced by Bebop *States* (see :ref:`sec-states`)

