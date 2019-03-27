
********
Cmake
********

Cmake getting started
======================

Example
--------

.. code-block:: cmake
    :caption: Cmake for OpenCV

    cmake_minimum_required(VERSION 2.8)
    project( DisplayImage )
    find_package( OpenCV REQUIRED )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    add_executable( DisplayImage DisplayImage.cpp )
    target_link_libraries( DisplayImage ${OpenCV_LIBS} )

.. code-block:: cmake
    :caption: Cmake for Freenect

    cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
    project("Freenect demo")

    find_package(OpenCV REQUIRED)
    find_package(GLUT REQUIRED)

    find_package(Threads REQUIRED)
    find_package(libfreenect REQUIRED)

    include_directories("/usr/include/libusb-1.0/" "/usr/local/include/libfreenect/" ${PROJECT_SOURCE_DIR}/src )

    add_executable(maincpp src/main.cpp)
    target_link_libraries(maincpp  ${OpenCV_LIBS}
                                ${CMAKE_THREAD_LIBS_INIT}
                                ${FREENECT_LIBRARIES})

Variables
----------
 CMake lets you create text variables. For this purpose, the set() command is used. For instance, to create a variable MY_VAR with the value "hello", we write:

    set (MY_VAR "hello")

To refer to a variable later on, simply enclose it in the round braces prepended by a dollar sign:

    set (OTHER_VAR "${MY_VAR} world!")

Specifying header search paths
-------------------------------
 Header search paths are important for both the compiler (so that it knows where to search for headers) but also for CLion, since the IDE can index the relevant directories and provide code completion and navigation facilities on #include statements.

The compiler searches the headers in several predefined locations that are specific to the operating system being used. In addition, you can specify further directories with header files using CMake’s include_directories() command:

    include_directories( ${MY_SOURCE_DIR}/src )

where MY_SOURCE_DIR is the desired directory.

You can control the order of inclusion of the directories with additional BEFORE and AFTER keywords. The default behavior is to append the include directory with the current item:

    include_directories( BEFORE ${MY_SOURCE_DIR}/src )

Setting language standard
--------------------------

To enable a particular language standard, specify the desired standard in CMAKE_CXX_VARIABLE, as follows:

    set(CMAKE_CXX_STANDARD 11)  # enable C++11 standard

If you want to set up additional compiler-related settings, specify them in CMAKE_CXX_FLAGS:

    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall")

The above command instructs the compiler to show all the compiler errors and warnings. To get the later versions of the compiler, use -std=c++1y, -std=c++1z and so on.

Adding build targets
---------------------

A build target defines an executable or a library that the CMake script helps you build. A script can have more than one build target. To create an executable binary, you use the add_executable() command, i.e.,

    add_executable (my_executable my_source1.cpp my_source2.cpp)

where my_executable is the target name and ``*.cpp`` files are the source files needed to construct the final executable.

If you want to build a library instead, you can use the add_library() command:

    add_library (my_library STATIC|SHARED|MODULE my_source.cpp)

There are three types of library that you can build:

    STATIC builds a static library, i.e. a library that gets embedded into whichever executable decides to use it.

    SHARED builds a shared library (.so on Linux, .dll on Windows).

    MODULE builds a plugin library — something that you do not link against, but can load dynamically at runtime.

Including libraries
---------------------

Here’s how you actually make use of a library. As the first step, you need to instruct the compiler to find a desired library and its components:
find_package (my_library COMPONENTS REQUIRED component1 component2  OPTIONAL_COMPONENTS opt_component)
In this example, option REQUIRED determines component1 and component2 as the mandatory components for a project. The component opt_component is qualified as an optional one; this means, that compilation can proceeded regardless of whether the optional component is actually available.

Next up, you need to link an executable to the located library:
target_link_libraries (my_target my_library another_library)
Note, that target_link_libraries() shall be placed after add_executable() command.

Using Boost
-------------

The Boost libraries are the most popular C++ library set. While many libraries are header-only (meaning you don’t need to perform any linking, just include a few files), some libraries do, in fact, require compilation and linking.

So let’s get started by including the files. For this, you can use CLion’s incboost live template, which will generate the following:
find_package(Boost)
IF (Boost_FOUND)
include_directories(${Boost_INCLUDE_DIR})
endif()

Now, when it comes to libraries, Boost offers you options for both static and dynamic linking. There are also additional options, such as whether or not you want to enable multithreading (feel free to read up on this option, since it doesn’t actually make code parallel). Also, you can cherry-pick which Boost components you want:
set (Boost_USE_STATIC_LIBS OFF) # enable dynamic linking
set (Boost_USE_MULTITHREAD ON)  # enable multithreading
find_package (Boost COMPONENTS REQUIRED chrono filesystem)
And finally, to get the libraries, you use the good old target_link_libraries() command:
target_link_libraries (my_target my_library another_library)

An alternative approach to using libraries is to use an external dependencies manager that can use CMake to configure and build projects.

Including sub-projects
-----------------------

Projects can have dependencies to other projects. CMake does not have the notion of a ‘solution’ — instead, it allows you to define dependencies between projects. Typically, you want to organize your multi-project workspace such that:

    When opening the main project, the IDE shall open all the dependant projects as well.

    All the settings of the main project shall be applied to the dependant projects automatically.

    All the smart features (e.g refactoring, code completion, etc) shall take effect in all the projects.

The above points can be achieved with proper configuration of the CMakeLists file. Essentially, you need to organize your projects as a tree of subdirectories with the top-level CMakeLists.txt file in the root directory. Then, each subdirectory of that tree shall represent a single sub-project and contain its own CMakeLists.txt file, which is then included to the main project in the top-level CMakeLists.txt file, as follows:

    add_subdirectory (project1) # including project1 into the main project
    add_subdirectory (project2) # including project2 into the main project

Setting custom build types
---------------------------

You can extend the list of available build types with custom ones by setting them explicitly in CMakeLists.txt.

To adjust the list of build types, specify the CMAKE_CONFIGURATION_TYPES variable, for example

.. code-block:: cmake
    :caption: CMAKE_CONFIGURATION_TYPES 

    cmake_minimum_required(VERSION 3.6)
    project(exampleProject)

    # setting two custom build types
    # variable CMAKE_CONFIGURATION_TYPES shall be defined prior to other definitions:
    set(CMAKE_CONFIGURATION_TYPES "CustomType1;CustomType2" CACHE STRING "" FORCE)

    set(CMAKE_CXX_STANDARD 11)                     # setting the language standard
    set(SOURCE_FILES main.cpp)
    add_executable(exampleProject ${SOURCE_FILES})

