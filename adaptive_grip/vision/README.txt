The programs in the directory you are currently in (vision) are built using CMake.
You will need CMake to build any of the executables (along with a working g++, etc.
CMake will let you know if you are missing any libraries; taking care of that is 
outside the scope of this guide.)

Each folder in this directory houses a single executable. The only files that should
ever be committed in a directory are:
   Source files (*.cpp)
   CMake configuration files ("CMakeLists.txt").

To build a program: (I'll use the "pcl_example" program as an example)

   cd into its directory

   > cd pcl_example

   create a folder called 'build'. (If you know what you're doing, and want to make 
   a debug executable, create a folder called 'debug'. Reason:
   Please only use the 'build'  and 'debug' names, so the wrong files don't get committed.
   I have the .gitignore file in this directory set to ignore folders named 'build' and 'debug')

   > mkdir build

   cd into your new build folder

   > cd build

   Invoke CMake. Use the CMakeLists.txt file the directory one level up.

   > cmake ..

   If this was successful, it should have generated a Makefile. Execute 'make'...

   > make

   This should put an executable in the folder you are currently in. In the case of
   the example I'm using, that executable is named `openni_grabber`. To run it
   simply enter its name into the shell.

   > ./openni_grabber

