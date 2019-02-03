#include "../include/kinect_to_gantry.h"

bool move_robot_home() {
    robot.home();

    // CURRENTLY THE ROBOT HAS A 'HOMED' PARAMETER THAT IS SET WITH THE HOME METHOD, BUT WE CAN'T GET ACCESS TO IT TO CHECK WHETHER IT'S ACTUALLY HOMED
    return true;
}
bool move_robot_to_pos(const float x_pos, const float y_pos) {

    // ((x, y, z, dim4, dim5, dim6), speed)
    robot.moveTo(RobotPosition(x_pos, y_pos, 0, 180, 35, -90), 1.0);

    return true;
}
bool move_robot_continuously(const Dimension& dim, const Direction& dir) {
    cout << "MSG from kinect_to_gantry.cpp: move_robot_continuously NOT IMPLEMENTED YET" << endl;
    return true;
}