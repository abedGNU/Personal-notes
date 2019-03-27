
A Makefile template for simple C projects
9 November 2014

Large programming projects may require elaborate build systems, but for a simple project consisting of a single executable target and handful of C source files, a single Makefile is often all that is necessary. If you use GNU Make and a C compiler capable of emitting dependency rules (both GCC and Clang can do this, and no doubt others can as well), then a single generic Makefile template can be used for multiple projects by changing only a few variable definitions.1 Over time I’ve developed such a template, which I present here with the hope that others will find it useful.

At the top of the template are variables with project-specific values. These are typically the only lines that need to be modified when starting a new project. The PROG variable holds the name of the executable target to be built, and SRCS holds the list of source files needed to build it. Header files are not listed here, because they are handled automatically. If an external library is needed, then the CFLAGS and LDFLAGS variables can be modified as well.

PROG := program
SRCS := main.c util.c

CC      := cc
CFLAGS  := -Wall -Wextra -Werror
LDFLAGS :=

In order to build the executable target, all the source files need to be compiled into object files, which are then linked together. The list of object files can be created from the list of source files by using a substitution reference. Keeping the list of object files in a variable makes it easy to write the rule for the executable target, and also the rule for cleaning up generated files. Another substitution reference is used to create a list of “.d” files in the DEPS variable. These files are created by the compiler (with the -MMD flag) alongside each object file, and contain dependency information in the form of Makefile rules. The dependencies include the C source file, along with any header files that it includes. By getting this information directly from the compiler, we’re spared from having to manually list dependencies for each file in the project. This is the key to keeping the Makefile as generic as possible.

OBJS   = $(SRCS:.c=.o)
DEPS   = $(SRCS:.c=.d)

.PHONY: all
all: $(PROG)

$(PROG): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -MMD -c $< -o $@

-include $(DEPS)

The -include $(DEPS) line includes the dependency files that were generated as a side effect of the %.o: %.c rule used to compile the object files. The first time the project is built, there won’t be any dependency files to include, but that’s okay because every source file will have to be compiled anyway. On subsequent runs, the dependency information will be used to determine which object files are out of date. And even better, when the object files are rebuilt, their dependency files will also be updated for the next run.

The final section of the Makefile has targets for cleaning up all the generated files. The clean target removes object and dependency files. The cleaner target depends on clean and also removes the executable file.

.PHONY: clean cleaner
clean:
	rm -f $(OBJS) $(DEPS)

cleaner: clean
	rm -rf $(PROG)

Below is the complete Makefile template, which I have placed in the public domain. Feel free to modify and use it for your own projects.

PROG := program
SRCS := main.c util.c

CC      := cc
CFLAGS  := -Wall -Wextra -Werror
LDFLAGS :=

OBJS   = $(SRCS:.c=.o)
DEPS   = $(SRCS:.c=.d)

.PHONY: all
all: $(PROG)

$(PROG): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -MMD -c $< -o $@

-include $(DEPS)

.PHONY: clean cleaner
clean:
	rm -f $(OBJS) $(DEPS)

cleaner: clean
	rm -rf $(PROG)
