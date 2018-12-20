# Compiler for C++
CC = g++

# Compilation Flags
CFLAGS = -O2 -Wall -Wextra

# Target executable
TARGET = robot.out

# Required src files
SRC_FILES = ./src/Robot.cpp ./src/RobotController.cpp ./src/RobotLimits.cpp ./src/RobotPosition.cpp ./src/CRScomm.cpp

# Example program to be compiled
ifndef PROGRAM_FILE
	PROGRAM_FILE = ./examples/robot_home.cpp
endif

all: $(TARGET)

$(TARGET):
	$(CC) $(CFLAGS) -o ./build/$(TARGET) -I ./src $(PROGRAM_FILE) $(SRC_FILES)
	@echo Build done! $(PROGRAM_FILE) was compiled. $(TARGET) located at ./build/$(TARGET)

clean:
	rm ./build/*
	@echo Clean done!