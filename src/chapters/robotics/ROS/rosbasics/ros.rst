
*******************
ROS basics
*******************

Overview
========

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

Like Arduino, Eclispe and other IDE, to develop in ROS you need to create a workspace.
A workspace mainly contain the source code folder and the compiled code and eventually other folders and files.

In order to create a workspace in ROS, ``catkin`` tools are used. These tools automate some job in Cmake.
There are 3 ways to use catkin. The first one is the most used. The second one ``catkin_tools`` is under development. The third one ``catkin_simple`` can be used in order to simplify the CMakeLists.txt file.

Before using the catkin commands, you need to create a workspace directory and a ``src`` subdirectory. You can call the workspace as you want. The directories can be created using the ``terminal`` or using the desktop utilities. Usually we will use the terminal. If you want to use an IDE, ``RoboWare Studio`` and ``Qt`` plugin ``ros_qtc_plugin`` are good choices.

catkin
------

We will create a workspace called ``catkin_ws`` and a subdirectory ``src`` in the home directory of linux. The following code will create a folder called ``src``. The ``-p`` option will create the folder ``catkin_ws`` if it doesn't exist ::

  mkdir -p ~/catkin_ws/src

Navigate to the ``src`` folder and initialize the workspace. The command ``catkin_init_workspace`` should be executed within the folder ``src`` ::

  cd ~/catkin_ws/src
  catkin_init_workspace

Go back to the workspace directory and type ```catkin_make``` in order to build the source code ::

  cd ~/catkin_ws/
  catkin_make

In order to build the worksapce after any modification of the source code, the ``catkin_make`` command should be run from the workspace directory.

catkin tools
------------

This is a python package that can be used to create and build a ROS workspace. It is not installed by default. Run the following commnad to install it ::

  sudo apt-get install python-catkin-tools

The documentation of this package can be found:

  https://github.com/catkin/catkin_tools

  http://catkin-tools.readthedocs.org/

Once the workspace and src folders are created, to initilize the workspace run the command ``catkin init`` from the workspace directory not from the ``src`` ::

  cd ~/catkin_ws/
  catkin init

To build the worksapce run ::

  catkin build

The command ``catkin build`` will initilize the workspace if it was not initilized by the ``catkin init`` command.
The ``catkin build`` command can be run from any subdirectory of the workspace.

catkin simple
-------------

Wokspace subdirectory
---------------------

Once is build the workspace will contain the ``src``, ``build`` and ``devel`` folders. These folders are create by all the prvious methods.

Environment variable
--------------------

The catkin_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder. Additionally, if you look in your current directory you should now have a ``build`` and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.\*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup.\*sh file:

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

Catkin
-------------

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

catkin_tools
-------------

catkin_simple
-------------
