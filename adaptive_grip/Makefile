# ===================================================
# 	Name		: Makefile
# 	Author	: AN
#	Purpose	: Makefile used to build:
#						- Object files from 'original' code
#						- New Executables
#
#
#  #### NOTE THAT I AM GOING TO DEPRECATE THIS
#  #### IN FAVOUR OF USING CMAKE
# ===================================================


# Command-Line Defines (none right now)
DEFINES=

# Directory housing the 'original' source code files.
GROUP3=/Users/amnicholas/Documents/ELEC490/Group3
ORIGDIR=$(GROUP3)/original
ORIGINAL_SRC=$(ORIGDIR)/src
EXT_SRC=src

# TODO this is a hack
PCL=-I

INCLUDE=-I$(ORIGDIR)/headers -I$(LIBDIR) -Iheaders

# Location where object files will be placed
BASE_OBJDIR=base_objects
EXT_OBJDIR=ext_objects
OBJDIR=objects

# =============================================
# New Source Library
# =============================================
SRC=src

# =============================================
# Programs
# =============================================
PROGRAMS=programs

# =============================================
# Libraries (readline)
# =============================================
#LIBDIR=/home/robot/Documents/Group3/libraries
LIBDIR=~/Documents/ELEC490/libraries/readline-master

# Need this for extended initializer lists
# e.g. some_function_call( {where, I , declare, "a struct", "in place"});
FLAGS=-std=c++11

# =============================================
# Objects
# =============================================
COMMON_OBJECTS_BASE= \
	$(BASE_OBJDIR)/Robot.o \
	$(BASE_OBJDIR)/CRScomm.o \
	$(BASE_OBJDIR)/RobotController.o \
	$(BASE_OBJDIR)/RobotLimits.o \
	$(BASE_OBJDIR)/RobotPosition.o

# =============================================
# Newer Objects
# =============================================
COMMON_OBJECTS_EXT= \
	$(EXT_OBJDIR)/Camera.o
#$(EXT_OBJDIR)/RobotExt.o \
	$(EXT_OBJDIR)/Camera.o




# ==================================================
# Object File Recipe
# ==================================================
$(BASE_OBJDIR)/%.o : $(ORIGINAL_SRC)/%.cpp $(wildcard headers/%.h)
	g++ -g $(DEFINES) $(INCLUDE) -c $< -o $@

$(EXT_OBJDIR)/%.o : $(EXT_SRC)/%.cpp $(wildcard headers/%.h)
	g++ -g $(DEFINES) $(INCLUDE) $(PCL) -c $< -o $@

# ==================================================
# Default: Make all of the executables
# ==================================================
all : test_homing basic_shell interface_update

# ==================================================
# Recipe for basic_shell executable
# ==================================================
basic_shell : $(PROGRAMS)/basic_shell.cpp $(COMMON_OBJECTS_BASE)
	make --directory $(ORIGDIR)
	g++ -g $(DEFINES) $(FLAGS) $(INCLUDE) $(PROGRAMS)/basic_shell.cpp -c -o $(BASE_OBJDIR)/basic_shell.o
	g++ -g -w $(DEFINES)  $(BASE_OBJDIR)/basic_shell.o $(COMMON_OBJECTS_BASE) \
	-std=c++11 \
	$(LIBDIR)/*.o \
	-ltermcap \
	-o exec/basic_shell

# ==================================================
# Recipe for interface_update executable
# ==================================================
interface_update : $(PROGRAMS)/interface_update.cpp $(COMMON_OBJECTS_BASE)
	g++ -g $(DEFINES) $(INCLUDE) $(PROGRAMS)/interface_update.cpp -c -o $(BASE_OBJDIR)/interface_update.o
	g++ -g -w $(DEFINES)  $(BASE_OBJDIR)/interface_update.o $(COMMON_OBJECTS_BASE) \
	-std=c++11 \
	$(LIBDIR)/*.o \
	-ltermcap \
	-o exec/interface_update

# ==================================================
# Recipe for test_homing executable
# ==================================================
test_homing : $(PROGRAMS)/test_homing.cpp $(COMMON_OBJECTS_BASE) $(COMMON_OBJECTS_EXT)
	g++ -g $(DEFINES) $(INCLUDE) $(PROGRAMS)/test_homing.cpp -c -o $(BASE_OBJDIR)/test_homing.o
	g++ -g -w $(DEFINES)  $(BASE_OBJDIR)/test_homing.o $(COMMON_OBJECTS_BASE) $(COMMON_OBJECTS_EXT) \
	-std=c++11 \
	$(LIBDIR)/*.o \
	-ltermcap \
	-o exec/test_homing
	
# =================================================
# Make and launch basic_shell program
# =================================================
shell_target: basic_shell
	exec/basic_shell

# =================================================
# Make and launch interface_update program
# =================================================
interface_target: interface_update
	exec/interface_update
	
# =================================================
# Clean everything
# =================================================
clean:
	make --directory $(ORIGDIR) clean
	rm -rf base_objects/*
	rm -rf ext_objects/*
	rm -rf exec/*
	rm -rf objects/*
