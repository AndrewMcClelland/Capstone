# MAKEFILE FOR COMPILING GANTRY ROBOT CODE

# Compiler for C++
CC = g++

# Compilation Flags
CFLAGS = -O2 -Wall -Wextra

# Target executable
TARGET = robot_shell
#TARGET = robot_home

# Required src files
SRC_FILES = ./src/Robot.cpp ./src/RobotController.cpp ./src/RobotLimits.cpp ./src/RobotPosition.cpp ./src/CRScomm.cpp

# Example program to be compiled
ifndef PROGRAM_FILE
	PROGRAM_FILE = ./robot_shell.cpp
#	PROGRAM_FILE = ./examples/robot_home.cpp	
endif

all: $(TARGET)

$(TARGET):
	$(CC) $(CFLAGS) -o ~/Desktop/Capstone/build/gantry/$(TARGET) -I ./include $(PROGRAM_FILE) $(SRC_FILES) -lreadline
	@echo Build done! $(PROGRAM_FILE) was compiled. $(TARGET) located at ~/Desktop/Capstone/build/gantry/$(TARGET)

clean:
	rm ./build/*
	@echo Clean done!
