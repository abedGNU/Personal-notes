
******************
C/C++ compilation
******************

Gcc
=====

Gcc getting started
--------------------

.. code-block:: c
    :caption: C program

    // hello.c
    #include <stdio.h>
    void main(void)
    {
        printf ("hello world!\n");
    }

    // compile the source file hello.c. Generate the output file a.out
    gcc hello.c
    
    // give the output file a name
    gcc hello.c -o hello
    chmod a+x hello
    ./hello 

.. code-block:: c++
    :caption: C++ program

    // hello.cpp
    #include <iostream>
    using namespace std;
    
    int main() {
    cout << "Hello, world!" << endl;
    return 0;
    }

    g++ -o hello hello.cpp
    chmod a+x hello
    ./hello

Gcc compilation process
------------------------

Various examples
-----------------

.. code-block:: c
    :caption: Some compiler options

    -o: specifies the output executable filename.
    -Wall: prints "all" Warning messages.
    -g: generates additional symbolic debuggging information for use with gdb debugger.
    -v: verbose mode
    -S: compile into assenbly
    -D: macro definition. -Dname=value

.. code-block:: c
    :caption: Examples

    gcc -Wall -g -o hello hello.c

    gcc -v -o hello hello.c

    // compile a program with 2 source files
    g++ -o myprog file1.cpp file2.cpp
    // or
    g++ -c file1.cpp
    g++ -c file2.cpp
    g++ -o myprog file1.o file2.o

.. code-block:: c
    :caption: Cross compilation for ARM Cortex-M

    arm-none-eabi-gcc -c test.c -mthumb -mcpu=cortex-m4

    arm-none-eabi-ld -o image.elf object1.o object2.o -T linker_script.ld -Map=map_file.map

Analyse the compiled file
-------------------------

Libraries
----------

Headers .h
^^^^^^^^^^^

Static Library .lib .a
^^^^^^^^^^^^^^^^^^^^^^^


Shared Library .dll .so
^^^^^^^^^^^^^^^^^^^^^^^^

pkg-config
------------

gcc -o test test.c `pkg-config --libs --cflags glib-2.0`

pkg-config --libs --cflags glib-2.0

Environment Variables
----------------------

Debugging
==========

Gdb
----

Gcc MCU
=======

ARM Cortex-M section in MCU

