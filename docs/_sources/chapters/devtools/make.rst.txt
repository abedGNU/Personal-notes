
**********
GNU Make
**********

Gnu Make getting started
=========================

.. code-block:: make
    :caption: Syntax of Rules

    target: pre-req-1 pre-req-2 ...
    	command

.. code-block:: c
    :caption: C program

    // hello.c
    #include <stdio.h>
    
    int main() {
        printf("Hello, world!\n");
        return 0;
    }

.. code-block:: make
    :caption: makefile simple example

    hello: hello.o
        gcc -o hello hello.o

    hello.o: hello.c
        gcc -c hello.c

Syntax
=======

Comments : #

Continuation: \

Phony targets : all, clean, install, ...

Variables :  $(CC), $(CC_FLAGS), $@, $^

.. code-block:: make
    :caption: Variables

    files = file1 file2
    some_binary: $(files)
        echo "Look at this variable: " $(files)

Automatic Variables
====================

.. code-block:: make
    :caption: Automatic Variables

    $@: the target filename.
    $*: the target filename without the file extension.
    $<: the first prerequisite filename.
    $^: the filenames of all the prerequisites, separated by spaces, discard duplicates.
    $+: similar to $^, but includes duplicates.
    $?: the names of all prerequisites that are newer than the target, separated by spaces.

.. code-block:: make
    :caption: Example with automatic variables

    all: hello
    
    # $@ matches the target; $< matches the first dependent
    hello: hello.o
        gcc -o $@ $<

    hello.o: hello.c
        gcc -c $<
        
    clean:
        rm hello.o hello

Using Implicit Rules
=======================
To allow make to find a customary method for updating a target file, all you have to do is refrain from specifying recipes yourself. Either write a rule with no recipe, or don’t write a rule at all. Then make will figure out which implicit rule to use based on which kind of source file exists or can be made.

For example, suppose the makefile looks like this:

foo : foo.o bar.o
        cc -o foo foo.o bar.o $(CFLAGS) $(LDFLAGS)

Because you mention foo.o but do not give a rule for it, make will automatically look for an implicit rule that tells how to update it. This happens whether or not the file foo.o currently exists.

If an implicit rule is found, it can supply both a recipe and one or more prerequisites (the source files). You would want to write a rule for foo.o with no recipe if you need to specify additional prerequisites, such as header files, that the implicit rule cannot supply.

Each implicit rule has a target pattern and prerequisite patterns. There may be many implicit rules with the same target pattern. For example, numerous rules make ‘.o’ files: one, from a ‘.c’ file with the C compiler; another, from a ‘.p’ file with the Pascal compiler; and so on. The rule that actually applies is the one whose prerequisites exist or can be made. So, if you have a file foo.c, make will run the C compiler; otherwise, if you have a file foo.p, make will run the Pascal compiler; and so on.

Of course, when you write the makefile, you know which implicit rule you want make to use, and you know it will choose that one because you know which possible prerequisite files are supposed to exist. See Catalogue of Built-In Rules, for a catalogue of all the predefined implicit rules.

Above, we said an implicit rule applies if the required prerequisites “exist or can be made”. A file “can be made” if it is mentioned explicitly in the makefile as a target or a prerequisite, or if an implicit rule can be recursively found for how to make it. When an implicit prerequisite is the result of another implicit rule, we say that chaining is occurring. See Chains of Implicit Rules.

In general, make searches for an implicit rule for each target, and for each double-colon rule, that has no recipe. A file that is mentioned only as a prerequisite is considered a target whose rule specifies nothing, so implicit rule search happens for it. See Implicit Rule Search Algorithm, for the details of how the search is done.

Note that explicit prerequisites do not influence implicit rule search. For example, consider this explicit rule:

foo.o: foo.p

The prerequisite on foo.p does not necessarily mean that make will remake foo.o according to the implicit rule to make an object file, a .o file, from a Pascal source file, a .p file. For example, if foo.c also exists, the implicit rule to make an object file from a C source file is used instead, because it appears before the Pascal rule in the list of predefined implicit rules (see Catalogue of Built-In Rules).

If you do not want an implicit rule to be used for a target that has no recipe, you can give that target an empty recipe by writing a semicolon (see Defining Empty Recipes). 

Using Wildcard Characters in File Names
==========================================
A single file name can specify many files using wildcard characters. The wildcard characters in make are ‘*’, ‘?’ and ‘[…]’, the same as in the Bourne shell. For example, *.c specifies a list of all the files (in the working directory) whose names end in ‘.c’. 

Wildcard expansion is performed by make automatically in targets and in prerequisites. In recipes, the shell is responsible for wildcard expansion. In other contexts, wildcard expansion happens only if you request it explicitly with the wildcard function. 

Wildcard Examples
^^^^^^^^^^^^^^^^^^^^

Wildcards can be used in the recipe of a rule, where they are expanded by the shell. For example, here is a rule to delete all the object files:

clean:
        rm -f *.o

Wildcards are also useful in the prerequisites of a rule. With the following rule in the makefile, ‘make print’ will print all the ‘.c’ files that have changed since the last time you printed them:

print: *.c
        lpr -p $?
        touch print

This rule uses print as an empty target file; see Empty Target Files to Record Events. (The automatic variable ‘$?’ is used to print only those files that have changed; see Automatic Variables.)

Wildcard expansion does not happen when you define a variable. Thus, if you write this:

objects = *.o

then the value of the variable objects is the actual string ‘*.o’. However, if you use the value of objects in a target or prerequisite, wildcard expansion will take place there. If you use the value of objects in a recipe, the shell may perform wildcard expansion when the recipe runs. To set objects to the expansion, instead use:

objects := $(wildcard *.o)

The Function wildcard
^^^^^^^^^^^^^^^^^^^^^^^
Wildcard expansion happens automatically in rules. But wildcard expansion does not normally take place when a variable is set, or inside the arguments of a function. If you want to do wildcard expansion in such places, you need to use the wildcard function, like this:

$(wildcard pattern…)

This string, used anywhere in a makefile, is replaced by a space-separated list of names of existing files that match one of the given file name patterns. If no existing file name matches a pattern, then that pattern is omitted from the output of the wildcard function. Note that this is different from how unmatched wildcards behave in rules, where they are used verbatim rather than ignored (see Wildcard Pitfall).

One use of the wildcard function is to get a list of all the C source files in a directory, like this:

$(wildcard *.c)

We can change the list of C source files into a list of object files by replacing the ‘.c’ suffix with ‘.o’ in the result, like this:

$(patsubst %.c,%.o,$(wildcard *.c))

(Here we have used another function, patsubst. See Functions for String Substitution and Analysis.)

Thus, a makefile to compile all C source files in the directory and then link them together could be written as follows:

objects := $(patsubst %.c,%.o,$(wildcard *.c))

foo : $(objects)
        cc -o foo $(objects)

(This takes advantage of the implicit rule for compiling C programs, so there is no need to write explicit rules for compiling the files. See The Two Flavors of Variables, for an explanation of ‘:=’, which is a variant of ‘=’.) 

Makefile templates
==================

Makefile example 1
--------------------

.. code-block:: make
    :caption: A makefile for 99% of your programs

    src = $(wildcard *.c)
    obj = $(src:.c=.o)

    LDFLAGS = -lGL -lglut -lpng -lz -lm

    myprog: $(obj)
        $(CC) -o $@ $^ $(LDFLAGS)

    .PHONY: clean
    clean:
        rm -f $(obj) myprog

Makefile example 2
--------------------

.. code-block:: make
    :caption: Multiple source directories

    csrc = $(wildcard src/*.c) \
        $(wildcard src/engine/*.c) \
        $(wildcard src/audio/*.c)
        
    ccsrc = $(wildcard src/*.cc) \
            $(wildcard src/engine/*.cc) \
            $(wildcard src/audio/*.cc)
    obj = $(csrc:.c=.o) $(ccsrc:.cc=.o)

    LDFLAGS = -lGL -lglut -lpng -lz -lm

    mygame: $(obj)
        $(CXX) -o $@ $^ $(LDFLAGS)

Makefile example 3
--------------------

.. code-block:: make
    :caption: Makefile template

    CC      := -gcc
    CCFLAGS := -Wall
    LDFLAGS  := 
    BUILD    := ./build
    OBJ_DIR  := $(BUILD)/objects
    BIN_DIR  := $(BUILD)/bin
    TARGET   := program
    INCLUDE  := -Iinclude/ \
                -Iinclude/module1/
    SRC      :=                      \
    $(wildcard src/module1/*.c) \
    $(wildcard src/*.c)         \

    OBJECTS := $(SRC:%.c=$(OBJ_DIR)/%.o)

    all: build $(BIN_DIR)/$(TARGET)

    $(OBJ_DIR)/%.o: %.c
        @mkdir -p $(@D)
        $(CC) $(CCFLAGS) $(INCLUDE) -o $@ -c $<

    $(BIN_DIR)/$(TARGET): $(OBJECTS)
        @mkdir -p $(@D)
        $(CC) $(CCFLAGS) $(INCLUDE) $(LDFLAGS) -o $(BIN_DIR)/$(TARGET) $(OBJECTS)

    .PHONY: all build clean debug release

    build:
        @mkdir -p $(BIN_DIR)
        @mkdir -p $(OBJ_DIR)

    debug: CCFLAGS += -DDEBUG -g
    debug: all

    release: CCFLAGS += -O2
    release: all

    clean:
        -@rm -rvf $(OBJ_DIR)/*
        -@rm -rvf $(BIN_DIR)/*
