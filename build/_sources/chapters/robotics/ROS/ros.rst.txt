
*******************
ROS
*******************

.. contents:: Table of Contents

Overview
========



Installation
=============

.. code-block:: bash

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  // key for kinectic
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop-full
  sudo rosdep init
  rosdep update

It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

.. code-block:: bash

  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc


If you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.

.. code-block:: bash
  :caption: If you just want to change the environment of your current shell instead of the above you can type

    source /opt/ros/kinetic/setup.bash


.. code-block:: bash

  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential



Catkin workspace
================

Let's create and build a catkin workspace:

.. code-block:: bash
  :caption: Workspace settings

  	mkdir -p ~/catkin_ws/src

  	cd ~/catkin_ws/src

  	catkin_init_workspace

  	cd ~/catkin_ws/

  	catkin_make

The catkin\_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder. Additionally, if you look in your current directory you should now have a ``build`` and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.\*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup.\*sh file:

.. code-block:: bash

  // this will be valid only for the opened terminal
  	source devel/setup.bash

To make it permanent do the following:

.. code-block:: bash

	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
	source ~/.bashrc

To make sure your workspace is properly overlayed by the setup script, make sure ROS\_PACKAGE\_PATH environment variable includes the directory you're in.

.. code-block:: bash

  echo $ROS_PACKAGE_PATH

Creating a ROS Package
======================

.. code-block:: bash

  // This is an example, do not try to run this
  // catkin_create_pkg <package_name> [depend1] [depend2] [depend3]

.. code-block:: bash

	cd ~/catkin_ws/src
	// create a package
	catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
	// build all packages in the workspace
	cd ~/catkin_ws
	catkin_make

	//To add the workspace to your ROS environment you need to source the generated setup file
	. ~/catkin_ws/devel/setup.bash

ROS basics
==========

In this section we see the basic concepts in order to create a simple package containing two nodes.
Nodes are programs, a package is a collection of programs. ROS is based on nodes, that are programs that communicate with each others, send messages via topics. A topic is the bus where a message is sent.

The master, that can be run by ```rosrun```, provides naming and registration services to the rest of nodes in the ROS system. When a node publish an information, using a message structure, it publish it on a topic. A node must subscribe to a topic in order to read that information. The master track publishers and subscribers.

Messages are data types or data structures, one can use ROS messages, std\_msgs, or create new one. User defined messages are stored in files with .msg extensions in the msg folder in the src folder of the workspace.

Topics are only one way buses, it mean if a node publish a topic, it doesn't wait an answer. If a node need to receive a reply from another node, services should be used.

Node source files are stored in src, user messages are stored in msg folder, services are stored in srv folder.

Tutorial examples can be find in /opt/ros/kinetic/share/rsopy\_tutorial or roscpp\_tutorial.

On the website of ROS, in the section wiki you can find clear and well explained tutorials about the basics.

The following two tutorial can be found on https://github.com/ros/catkin_tutorials.git

Simple Publisher-Subscriber ROS program
=======================================

In this section we will make a package that contain 2 nodes. A publisher node (talker.cpp) and a subscriber node (listener.cpp).
These two source files should be created in the src folder of the package.

Append to the CMakeLists.txt of the package the following:

.. code-block:: cmake

  add_executable(talker src/talker.cpp)
  target_link_libraries(talker ${catkin_LIBRARIES})

  add_executable(listener src/listener.cpp)
  target_link_libraries(listener ${catkin_LIBRARIES})


This will create two executables, talker and listener, which by default will go into package directory of your devel space, located by default at ~/catkin\_ws/devel/lib/<package name>.

Publisher node
---------------

``listing/tutorial/talker.cpp``

Subscriber Node
--------------------

The
``listing/tutorial/listener.cpp``

Building nodes
--------------

Packages should be stored in workspace, if no work space is present, one should be created.
The following steps should be done:

	- Create and build a workspace
	- Source the package environment variable
	- Create a package
	- Copy or create node source file into the src folder of the package
	- Eventually create new messages and services
	- Add nodes, messages and services to the CMakeLists.txt of the package
	- Build the workspace

Listing.\ref{lstquickROS} show steps necessary to create in the home directory a workspace named catkin\_ws, create a package called first\_tutorial, and create two nodes in that package then build the workspace.

``label=lstquickROS listing/tutorial/quickROS``

This will create two executables, talker and listener, which by default will go into package directory of your devel space, located by default at ~/catkin\_ws/devel/lib/<package name>.

Run nodes
---------

.. code-block:: bash

  roscore

  rosrun beginner_tutorials talker      (C++)
  rosrun beginner_tutorials talker.py   (Python)

.. code-block:: bash

	rosrun beginner_tutorials listener     (C++)
	rosrun beginner_tutorials listener.py  (Python)


Useful ROS commands
-------------------

rosnode
rostopic
rosmsg

Simple Service and Client
=========================

User messages
=============

Services
========

Service node
-------------

``listing/tutorial/add_two_ints_server.cpp``

Client node
-----------

``listing/tutorial/add_two_ints_client.cpp``

Build the package
-----------------

``caption=CmakeLists with new messages and services, label=lstCMakeLists`` ``listing/tutorial/CMakeLists.txt``

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
