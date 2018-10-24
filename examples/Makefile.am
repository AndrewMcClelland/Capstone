CLEANFILES = *.log

bin_PROGRAMS		= robot_test robot_limits robot_controller robot_shell robot_home

#   Cannot get robot_limits or robot_home to compile - therefore have left out.

INCLUDES	= -I$(top_srcdir)/src

CPPFLAGS = -Wall -O2

robot_test_SOURCES = test.cpp
robot_test_LDADD	=  	$(top_srcdir)/src/libRobot.a

robot_limits_SOURCES = limits.cpp
robot_limits_LDADD	=  	$(top_srcdir)/src/libRobot.a \
					-lCalibAlt
	
robot_controller_SOURCES = controller.cpp
robot_controller_LDADD	=  	$(top_srcdir)/src/libRobot.a

robot_shell_SOURCES = robot_shell.cpp
robot_shell_LDADD = -lreadline -lcurses

#NOTE THAT ROBOT_HOME IS DEPENDANT ON THE CALIB_ALT LIBRARY!!
robot_home_SOURCES = robot_home.cpp
robot_home_LDADD = 	$(top_srcdir)/src/libRobot.a \
					-lCalibAlt
