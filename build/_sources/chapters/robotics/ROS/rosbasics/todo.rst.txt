*******************
TODO
*******************


Embedded systems
================

Arduino
--------

Installation
^^^^^^^^^^^^^
To install the ROS-Arduino interface write the following commands in the terminal:

.. code-block:: bash
  :caption: Arduino installation packages

  	// Installing the rosserial metapackage
  	sudo apt-get install ros-kinetic-rosserial
  	// Install the rosserial-arduino client package
  	sudo apt-get install ros-kinetic-rosserial-arduino


Download and install Arduino IDE. To use the serial port without root permissions:

.. code-block:: bash

	ls -l /dev/ttyACM*
	// or
	ls -l /dev/ttyUSB*
	//
	sudo usermod -a -G dialout <username>

In Arduino IDE set the Sketchbook location to /home/robot/arduino. Arduino should create a folder called libraries inside it.

.. code-block:: bash

	cd /home/robot/arduino/libraries
	// don t forget the dot at he end of the following commands, it indicate current directory
	rosrun rosserial_arduino make_libraries.py .

After these steps, you should find the ros\_lib voice in the examples of Arduino IDE.

Monitoring light using Arduino and ROS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``listing/arduino/ADC_modified.ino``

Running ROS serial server on PC
-------------------------------

.. code-block:: bash
  :caption: Running ROS serial server on PC

  	roscore

  	rosrun rosserial_python serial_node.py /dev/ttyACM0

  	rostopic list

  	rostopic echo /adc/adc0

  	rqt_plot adc/adc0

Ti LaunchPad
-------------

Raspberry pi
------------

Vision
======

Camera
-------

Opencv
-------

Object detection
----------------

sudo apt-get install ros-kinetic-find-object-2d
