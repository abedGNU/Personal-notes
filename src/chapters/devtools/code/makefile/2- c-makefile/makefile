
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
