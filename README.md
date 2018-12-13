# Remote Gesture Controlled Gantry Robot

## Overview
The purpose of this project is to remotely control an industrial Gantry robot ([like this!](http://img.directindustry.com/images_di/photo-g/26133-2818209.jpg)) using motion gestures. Using a fixed camera mounted above the Gantry robot, the total area of the robot's view will be projected for a user. The user will interact with this projection through motion gestures in front of a Kinect v2. The purpose is to allow a user to remotely control the Gantry robot through hand gestures to pickup and move items (identified through edge detection) within the Gantry robot's range of motion.

## Contributors
* [Andrew McClelland](https://github.com/AndrewMcClelland)
* Nikita Lipski
* Jonathon Malcolm
* Jack Caldwell

## Project Structure

    .
    ├── build                   # Compiled files
    ├── examples                # Sample/test code that tests functionality of robot
    ├── src                     # Source files
    ├── LICENSE
    ├── Makefile                # Build binaries using src/header files from ./src, example code from ./examples, and store output in ./build
    └── README.md
    
## Dependencies
* [OpenCV 3.4](https://opencv.org/releases.html)
* [libfreenect2](https://github.com/OpenKinect/libfreenect2)
  * Ubuntu drivers for Kinect v2

## Build Instructions
Makefile in root dir is used to build binaries using `make` command
* Test example that is compiled is sourced from the `./examples` directory (ex. `robot_home.cpp`)
  * _Note that `Makefile` needs to be modified to accept the correct example code file using the `PROGRAM_FILE` variable_
* src and header files are pulled from the `./src` directory
* Compiled binaries are built to the `./build` directory
* Use `make clean` command to remove __ALL FILES__ in `./build` (_be sure when using this!_)
