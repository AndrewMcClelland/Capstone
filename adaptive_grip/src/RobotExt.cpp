// TODO list
//    - Poll the robot to make sure it's homed


#include "RobotExt.h"

RobotExt::RobotExt(const std::string & device) :
   Robot(device)
{
   m_current_severity = DEBUG;
}

void
RobotExt::home(const bool& redo)
{
   // Execute homing sequence.
   
   // Home axes 1, 2, 4, 5, 6.
   runCmd("HOMESEQ 2");
   runCmd("HOMESEQ 1");

   runCmd("HOMESEQ 4");
   runCmd("HOMESEQ 5");
   runCmd("HOMESEQ 6");

// log(INFO, "Currently not homing axis 3.");
   log(WARNING, "Zero-cross homing axis 3.");

   // Have to zero-cross home axis 3. 
   runCmd("HOMEZC 3");


}


void 
RobotExt::log(info_t severity, const std::string message) 
{
   if (severity >= m_current_severity) 
   {
      if       (severity == DEBUG) std::cout << "DEBUG    :";
      else if  (severity == INFO ) std::cout << "INFO     :";
      else if  (severity == WARNING) std::cout << "WARNING  :";
      else if  (severity == ERROR) std::cout << "ERROR    :";
      else if  (severity == FATAL) std::cout << "FATAL    :";
      else                       std::cout << "OTHER    :";

      std::cout << message.c_str() << std::endl;
   }
}

void 
RobotExt::cRunCmd(const std::string cmd)
{
   // Do nothing right now.
   return;
}

void 
RobotExt::outputPos(info_t severity)
{
   // Get current position.
   const RobotPosition current    = currentPos(); 
   const    RobotPosition max     = currentLimits().max();
   const    RobotPosition min     = currentLimits().min();
   
   printf("%-10s | %-10s | %-10s | %-10s\n", "AXIS", "MIN", "CURRENT", "MAX");
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "X", min.x, current.x, max.x);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Y", min.y, current.y, max.y);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Z", min.z, current.z, max.z);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j4", min.j4, current.j4, max.j4);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j5", min.j5, current.j5, max.j5);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j6", min.j6, current.j6, max.j6);
}

bool 
RobotExt::scan()
{


// // Construct a vector of positions to scan with
// std::vector<RobotPosition> positions = std::vector<RobotPosition>();

// const int num_x_steps = 8;

// const double x_range = xyMax.x - xyMin.x;
// const double x_ste_amt     = x_range / num_x_steps;

// const double yMinAmt       = robot.currentLimits().min().y;
// const double yMaxAmt       = robot.currentLimits().max().y;

// RobotPosition x_step = RobotPosition(x_step_amt, 0.0, 0.0, 0.0, 0.0, 0.0);

// RobotPosition nextPos = xyMin + x_step;
// RobotPosition currentPos;

// bool           yMin          = true;

// do {

//    robot.moveTo(nextPos);

//    currentPos = robot.currentPos();

//    nextPos = currentPos + x_step;

//    if (yMin) {
//       nextPos.y = yMaxAmt;
//    } else {
//       nextPos.y = yMinAmt;
//    }

//    yMin = !yMin;

//    // If an object has been located under the Kinect, return true.
//    if (locate_under_kinect()) return true;

// } while ( nextPos.x <= limits.max().x );

   return false;

 
}
