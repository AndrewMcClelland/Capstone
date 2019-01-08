
#include "joystick.hh"
#include <unistd.h>
#include <stdio.h>
#include <cstdlib>
#include "Robot.h"
#include <string>
#include <sstream>

//#include "RobotExt.h"

using namespace std;


int main(int argc, char ** argv)
{
   /*
   // Create the robot object.
   RobotExt robot = RobotExt("/dev/cu.usbserial");

   // Home the robot.
   prompt("Homing the robot.");
   robot.home(false);

   // Move to minimum (X, Y)
   const RobotLimits limits = robot.currentLimits();

// const RobotPosition no_x_no_y = robot.getPos();

// no_x_no_y.x = 0;
// no_x_no_y.y = 0;

   
   
   // Move to maximum (X, Y)
// const RobotPosition min = limits.min() - no_x_no_y;
// const RobotPosition max = limits.max() - no_x_no_y;
   RobotPosition xyMin = robot.currentPos();
   xyMin.x = limits.min().x;
   xyMin.y = limits.min().y;

   RobotPosition xyMax = robot.currentPos();
   xyMax.x = limits.max().x;
   xyMax.y = limits.max().y;

   prompt("Moving to minimum position.");
   robot.moveTo(xyMin);

// prompt("Moving to maximum position.");
// robot.moveTo(xyMax);

   prompt("Scanning workspace.");
// robot.scan(); -- TODO put code in that function

   int            num_x_steps   = 8;
   const double   x_range       = xyMax.x - xyMin.x;
   const double   x_step_amt    = x_range / num_x_steps;


   const double   yMinAmt       = robot.currentLimits().min().y;
   const double   yMaxAmt       = robot.currentLimits().max().y;

   RobotPosition x_step = RobotPosition(x_step_amt, 0.0, 0.0, 0.0, 0.0, 0.0);

   RobotPosition nextPos = xyMin + x_step;
   RobotPosition currentPos;

   bool           yMin          = true;

   do {

      robot.moveTo(nextPos);

      currentPos = robot.currentPos();

      nextPos = currentPos + x_step;

      if (yMin) {
         nextPos.y = yMaxAmt;
      } else {
         nextPos.y = yMinAmt;
      }

      yMin = !yMin;

   } while ( nextPos.x <= limits.max().x );

   return false;
   */
}
