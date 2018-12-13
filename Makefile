# Compiler for C++
CC = g++

# Compilation Flags
CFLAGS = -O2 -Wall -Wextra

# Target executable
TARGET = robot.out

# Required src files
SRC_FILES = ./src/Robot.cpp ./src/RobotController.cpp ./src/RobotLimits.cpp ./src/RobotPosition.cpp ./src/CRScomm.cpp

#TODO - Fix this so that main file is taken in as argument to Make
# Example program to be compiled
PROGRAM_FILE = robot_home.cpp

all: $(TARGET)

$(TARGET):
	$(CC) $(CFLAGS) -o ./build/$(TARGET) -I ./src ./examples/$(PROGRAM_FILE) $(SRC_FILES)
	echo Build done! $(TARGET) located at ./build/$(TARGET)

clean:
	rm ./build/*
	echo Clean done!