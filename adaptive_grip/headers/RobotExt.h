#ifndef ROBOTEXT__H
#define ROBOTEXT__H

#include "Robot.h"

// Right now: same as Robot class, but with a re-done homing method.
class RobotExt : public Robot {

public :

   typedef enum {
      DEBUG       =   0,
      INFO        = 100,
      WARNING     = 200,
      ERROR       = 300,
      FATAL       = 400
   } info_t;


// Camera      m_camera;

   info_t m_current_severity;

   RobotExt(const std::string & device);

   void home(const bool& redo);

   void cRunCmd(const std::string cmd);

   void log(info_t severity, const std::string message);

   bool scan();

   void outputPos(info_t severity=INFO);

private:

   RobotPosition  _m_last_located_object;


};

#endif 
