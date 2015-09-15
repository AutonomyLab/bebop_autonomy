******************
Running the Driver
******************

You can run Bebop's ROS drivereither as a ROS `Nodelet <http://wiki.ros.org/nodelet>`_ or as a standalone ROS Node. The former is recommended if you intend to perform any kind of processing on Bebop's video stream.

.. note:: If you compile the driver form source, do not forget to source your catkin workspace prior to running the driver. (i.e. ``source ~/bebop_ws/devel/setup.[bash|zsh]``)

.. note:: Ensure that your Bebop's firmware is at least **2.0.29** and your computer is connected to Bebop's wireless network.

Running the driver as a Node
================================

The executable node is called ``bebop_driver_node`` and exists in ``bebop_driver`` package. It's recommended to run the Node in its own namespace and with default configuration. The driver package comes with a sample launch file ``bebop_driver/launch/bebop_node.launch`` which demonstrates the procedure.

.. code-block:: bash

  $ roslaunch bebop_driver bebop_node.launch


.. literalinclude::
  ../bebop_driver/launch/bebop_node.launch
  :language: XML
  :caption: bebop_node.launch

Running the driver as a Nodelet
===================================

To run the driver as a ROS Nodelet, you need to first run a Nodelet manager, then load the driver's Nodelet (``bebop_driver/BebopDriverNodelet``) in it, along with other Nodelets that need to communicate with the driver. `bebop_tools/launch/bebop_nodelet_iv.launch` is a sample launch file that demonstrates these steps by visualizing Bebop's video stream using an instance of `image_view/image <http://wiki.ros.org/image_view#image_view.2BAC8-diamondback.image_view.2BAC8-image>`_ Nodelet. Similar to `bebop_node.launch`, it also runs everything in its own namespace and loads the default configuration.

.. code-block:: bash

  $ roslaunch bebop_tools bebop_nodelet_iv.launch


.. literalinclude::
  ../bebop_tools/launch/bebop_nodelet_iv.launch
  :language: XML
  :caption: bebop_tools/launch/bebop_nodelet_iv.launch

.. literalinclude::
  ../bebop_driver/launch/bebop_nodelet.launch
  :language: XML
  :caption: bebop_driver/launch/bebop_nodelet.launch
