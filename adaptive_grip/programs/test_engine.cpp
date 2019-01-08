#include "Shell.h"
#include "Engine2.h"

   using std::cout;
   using std::endl;
   using std::string;
class TestEngineShell : public Shell {
   public:
   using Shell::callback;
   Engine2  eng;


   void home(arg_list args) {
      eng.m_robot->home(false);
   }

   void pos(arg_list args)
   {
      // get current position.
      const    RobotPosition current = eng.m_robot->currentPos();
      const    RobotPosition max     = eng.m_robot->currentLimits().max();
      const    RobotPosition min     = eng.m_robot->currentLimits().min();
      
      // cout << sprintf("") << endl;
      printf("%-10s | %-10s | %-10s | %-10s\n", "AXIS", "MIN", "CURRENT", "MAX");
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "X", min.x, current.x, max.x);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Y", min.y, current.y, max.y);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Z", min.z, current.z, max.z);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j4", min.j4, current.j4, max.j4);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j5", min.j5, current.j5, max.j5);
      printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j6", min.j6, current.j6, max.j6);
      
   }

   void move(arg_list args)
   {
      RobotPosition new_pos = eng.m_robot->currentPos();

      while (!args.empty()) {
         string axis = args.front();
         float value;
         args.pop_front();
         if (args.empty()) {
            cout << "Illegal command." << endl;
            return;
         } else {
            value = atof(args.front().c_str());
            args.pop_front();
         }

         if      (axis == "x")      new_pos.x = value;
         else if (axis == "y")      new_pos.y = value;
         else if (axis == "z")      new_pos.z = value;
         else {
            cout << "Unrecognized argument: <" << axis << ">" << endl;
            return;
         }
      }

      // Output the position to the console
      cout << "NEW POSITION:" << endl;
      cout << new_pos << endl;

      eng.m_robot->moveTo(new_pos);

      // Poll the buffer
      string reply;
      eng.m_robot->controller >> reply;

      cout << "REPLY:" << endl << reply << endl;

   }

   void calibrate(arg_list args)
   {
      eng.calibrate();
   }

   void display_all_objects(arg_list args)
   {
      typedef std::vector<WSObject>::iterator Iterator;
      printf("%-10s | %-10s | %-10s | %-10s\n", "ID", "X", "Y", "Obs");
      for (Iterator i = eng.m_objects->begin(); i != eng.m_objects->end(); i++)
      {
         printf("%10d | %-+10.2f | %-+10.2f | %-+10.2f\n", (*i).id, (*i).x_position, (*i).y_position, (*i).observation_distance);
      }
   }

   void find(arg_list args)
   {
      eng.locate((args.front() == "-h"));
   }

   void save_calibration(arg_list args)
   {
      const std::string calFile = "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/data/calibration.dat";
      cout << "Saving: " << endl <<  eng.m_cal.toString() << endl;
      if (eng.m_cal.toFile(calFile)) {
         cout << "Succesfully saved file." << endl;
      } else {
         cerr << "There was a problem saving the calibration file." << endl;
      }
   }
   void load_calibration(arg_list args)
   {
      eng.load();
   }

   // Test workspace scanning

   void clear_workspace()
   {
   }

   void set_fudge()
   {
   }

   void set_speed()
   {
   }

   void home5(arg_list args)
   {
      eng.m_robot->runCmd("RUN HOME5");
   }

   void bind_functions()
   {
      callback calibrate = boost::bind(&TestEngineShell::calibrate, this, _1);
      register_function(calibrate, "cal", "Calibrate the robot.");
      callback home = boost::bind(&TestEngineShell::home, this, _1);
      register_function(home, "home", "Home the robot.");
      callback home5 = boost::bind(&TestEngineShell::home5, this, _1);
      register_function(home5, "home5", "Home the robot.");
      callback move = boost::bind(&TestEngineShell::move, this, _1);
      register_function(move, "move", "Home the robot.");
      callback pos = boost::bind(&TestEngineShell::pos, this, _1);
      register_function(pos, "pos", "Home the robot.");
      callback find = boost::bind(&TestEngineShell::find, this, _1);
      register_function(find, "find", "Home the robot.");

      callback load = boost::bind(&TestEngineShell::load_calibration, this, _1);
      register_function(load, "load", "Home the robot.");
      callback save = boost::bind(&TestEngineShell::save_calibration, this, _1);
      register_function(save, "save", "Home the robot.");

      callback display = boost::bind(&TestEngineShell::display_all_objects, this, _1);
      register_function(display, "disp", "Home the robot.");
   }

   TestEngineShell() :
      Shell()
   {
      bind_functions();
   }


      
   
};

int main(int argc, char * argv[])
{
   TestEngineShell   ts;
   ts.run_shell();
   return SUCCESS;
}
