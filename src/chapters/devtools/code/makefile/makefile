#The PROG variable holds the name of the executable target to be built, 
#and SRCS holds the list of source files needed to build it.

PROG := program
SRCS := main.c util.c

CC      := cc

#Header files are not listed here, because they are handled automatically.
#If an external library is needed, then the CFLAGS and LDFLAGS variables can be modified as well.
CFLAGS  := -Wall -Wextra -Werror
LDFLAGS :=

# Substitution reference

OBJS   = $(SRCS:.c=.o)
DEPS   = $(SRCS:.c=.d)

.PHONY: all
all: $(PROG)

# $@: the target filename
# $^: the filenames of all the prerequisites, separated by spaces, discard duplicates.
$(PROG): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

# $<: the first prerequisite filename
%.o: %.c
	$(CC) $(CFLAGS) -MMD -c $< -o $@

# includes the dependency files that were generated as a side effect of the %.o: %.c rule used to compile the object files
-include $(DEPS)

.PHONY: clean cleaner
clean:
	rm -f $(OBJS) $(DEPS)

cleaner: clean
	rm -rf $(PROG)