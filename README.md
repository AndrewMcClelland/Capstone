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
    ├── build/                      # Compiled files
        ├── gantry/
        ├── gesture_recognition/
        ├── kinect/                 # Holds CMakeLists.txt for building kinect programs
            ├── bin/                # binaries for kinect programs
    ├── src/                        # Source files
        ├── gantry/
            ├── include/
            ├── src/
            ├── examples/           # Sample/test code that tests functionality of robot
            ├── Makefile            # Build binaries using src/header files from ./src, example code from ./examples, and store output in ./build
        ├── gesture_recognition/
            ├── include/
            ├── src/
        ├── kinect
            ├── include/
            ├── src/                # Source files for Kinect programs
    ├── LICENSE
    └── README.md
    
## Dependencies
* [OpenCV 3.4](https://opencv.org/releases.html)
* [libfreenect2](https://github.com/OpenKinect/libfreenect2)
  * Ubuntu drivers for Kinect v2

## Build Instructions
### Programs for Gantry robot (NOT Kinect programs)
Makefile in root dir is used to build binaries using `make` command
* Test example that is compiled is passed as `PROGRAM_FILE` parameter to `make` (ex. `make PROGRAM_FILE=./examples/robot_move_to.cpp`)
  * _If no `PROGRAM_FILE` is passed, then default compiled source file is `./examples/robot_home.cpp`_
* src and header files are pulled from the `./src` directory
* Compiled binaries are built to the `./build` directory
* Use `make clean` command to remove __ALL FILES__ in `./build` (_be sure when using this!_)

### Kinect v2 Programs
`CMakeLists.txt` in root dir is used to create a Makefile in `./build/` for Kinect programs from `./src/kinect/src/`
To build:
* Create `./src/kinect/src/main.cpp` that uses `libfreenect2` functionality
  * _Currently, `cmake` will only accept a program named `./src/kinect/src/main.cpp` and creates output binary as `./build/main` ... (will fix this later)_
* Navigate to `./build` and run `cmake .. && make`
* Output binary will be `./build/main`

## Gantry
* Turn on lower left power switch of controller box
* On controller buttons, hit `ESC` then `F1` to get it working for input from computer
* Change Gantry source code to the usb that the Gantry is connected to on computer

