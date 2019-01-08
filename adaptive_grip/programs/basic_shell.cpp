/*
   Name        :  basic_shell.cpp
   Author      :  Aaron
   Purpose     :  Implement a "shell" program that can be used to
                  send the Gantry robot some basic commands
*/

#include <string>
#include <list>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <sstream>

extern "C" {
#include <stdio.h>
#include "readline/readline.h"
#include "readline/history.h"
}

#include "RobotExt.h"

using namespace std;

namespace Direction {
   enum Dir {X, Y, Z, J4, J5, J6};
}

/*
   Typedefs/Structs
*/ 

typedef list<string>             arg_list;

// Void function that takes in an arg_list and RobotExt object
typedef void                     handler (arg_list& args, RobotExt& robot );
typedef struct {
   string                     name; // Function Name, to users
   handler  *                 func; // Handler Function Pointer
   string                     doc;  // Documentation
} Command;
typedef vector<Command>          command_list;
typedef command_list::iterator   command_iterator;

/*
   Global Variables
*/

// A file that will be used to 'log' events as they happen.
ofstream                Log("log/shell.log");

// Prompt
string                  Prompt = "$ ";

// Used to read command line
string                  cmdline;

// Set to true when we want to quit
bool quit      = false;  

// Set to true when the robot is homed.
bool homed     = false;  

list<string> split (const string& s);

command_list   commands;

// Declaration of 'handler' functions
handler        hl_help;
handler        hl_quit; 
handler        hl_move_xyz;
handler        hl_home_smart;


// Test new shit
handler        hl_aaron_test;

float convert_relative(Direction::Dir d, float f, RobotExt & robot);

/*
   Function       : split 
   Arguments      : 
      s: a string, with one or more words separated by spaces.
   Returns        : each space-delimited word in 's' is an entry
                    in the returned list.

*/
arg_list split (const string& s) 
{
   istringstream line(s);
   string word;
   list<string> words;
   while (line >> word) words.push_back(word);
   return words;
}

/*
   Function       : hl_help ('Help' Handler)
   Description    : Outputs a 'help' message.
   Arguments      : 
      - arg_list  : a list of arguments to the function that will be ignored.
      - robot     : a reference to the Gantry RobotExt that will be ignored.
   Returns        : void
*/
void hl_help(arg_list & args, RobotExt & robot)
{
   cout << "This is a help function!" << endl;
}

/*
   Function       : hl_quit ('Quit' Command Handler)
   Description    : Quits the shell.
   Arguments      : 
      - arg_list  : a list of arguments to the function that will be ignored.
      - robot     : a reference to the Gantry RobotExt that will be ignored.
   Returns        : void
*/
void hl_quit(arg_list & args, RobotExt & robot)
{
   cout << "You are a quitter!" << endl;
   quit = true;
}

/*
   Function       : hl_home_smart ('Smart Home' Command Handler)
   Description    : Homes the robot using HOMEZC instead of 'HOME' or 'RUN HOME5'
   Arguments      : 
      - arg_list  : A list of arguments to the function that will be ignored.
      - robot     : a reference to the Gantry RobotExt that will be ignored.
   Returns        : void
*/
void hl_home_smart(arg_list & args, RobotExt & robot)
{
   // Run through all six axes. Print an error message if the robot cannot home
   // on a particular axis.
   ostringstream     homezc_cmd;
   string            homezc_reply;
   size_t            error_msg_location;
   const string      error_msg("040-ARM POWER");

// for (int axis = 1; axis <= 6; axis++)
// {
//    homezc_cmd.str("HOMEZC ");
//    homezc_cmd.seekp(0, ios::end);
//    // Generate the "HOMEZC [AXIS]" command
//    homezc_cmd << axis;

//    // Run the command. Poll the controller buffer for an error 
//    // message; this will be present if arm power quits.
//    
//    // Block until the home message at least completes.
//    robot.controller << homezc_cmd.str() << RobotManipulators::block;

//    // Read the controller's reply from the homezc command.
//    robot.controller >> homezc_reply;
//    
//    // If the reply from the controller contains an 'ARM POWER' error
//    // message, it means the axis we tried to home did not home 
//    // successfully.
//    error_msg_location = homezc_reply.find(error_msg);
//    if (error_msg_location != string::npos) {
//       cout << "Lost arm power while homing axis " << axis << "." <<
//               " Arm was NOT succesffuly homed." << endl;
//       return;
//    } else {
//       cout << "Axis " << axis << " successfully homed." << endl;
//    }

//    // Set 'homed' to true.
//    homed = true;

// }
   robot.home(false);
}

/*
   Function       : hl_move_xyz
   Description    : Moves the robot to a set of x-, y-, and z- co-ordinates
                    (Gantry AXIS 1, AXIS 2, AXIS 3)
   Arguments      : 
      - arg_list  : A list of arguments in the format: X Y Z
                    where 'X', 'Y', and 'Z' are floating-point numbers.
      - robot     : a reference to the Gantry RobotExt 
   Returns        : void
*/
void hl_move_xyz(arg_list & args, RobotExt & robot)
{
   cout << "hl_move_xyz: Num args = " << args.size() << endl;

   // If we don't have three arguments, quit
   if (args.size() != 3) {
      cout << "hl_move_xyz: Need 3 arguments. Exiting..." << endl;
   }

   else {

      // Parse the floating-point numbers in these arguments
      double xpos, ypos, zpos;
      xpos = atof(args.front().c_str());
      args.pop_front();
      ypos = atof(args.front().c_str());
      args.pop_front();
      zpos = atof(args.front().c_str());
      args.pop_front();

      // Get the current position of the robot
      RobotPosition current_pos = robot.currentPos();

      // Set the new position of the robot.
      // Use the current position's "j4, j5, j6" for the new position's
      // "j4, j5, j6"
      RobotPosition new_pos = RobotPosition(xpos, ypos, zpos, 0, 0, 0);
      new_pos.j4 = current_pos.j4;
      new_pos.j5 = current_pos.j5;
      new_pos.j6 = -90.0;

      cout << "hl_move_xyz - Current Position: " << current_pos << endl;
      cout << "hl_move_xyz - Next    Position: " << new_pos << endl;

      robot.moveTo(new_pos);
   } 
}

void hl_move_to_position_float(arg_list & args, RobotExt & robot)
{
   RobotPosition new_pos = robot.currentPos();

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

   robot.moveTo(new_pos);

   // Poll the buffer
   string reply;
   robot.controller >> reply;

   cout << "REPLY:" << endl << reply << endl;

}

void hl_move_to_position_rel(arg_list & args, RobotExt & robot)
{
   RobotPosition new_pos = robot.currentPos();

   while (!args.empty()) {
      string      axis = args.front();
      float       value;

      args.pop_front();
      if (args.empty()) {
         cout << "Illegal command." << endl;
         return;
      } else {
         value = atof(args.front().c_str());
         args.pop_front();
      }

      if      (axis == "x")      new_pos.x = convert_relative(Direction::X, value, robot);
      else if (axis == "y")      new_pos.y = convert_relative(Direction::Y, value, robot);
      else if (axis == "z")      new_pos.z = convert_relative(Direction::Z, value, robot);
      else {
         cout << "Unrecognized argument: <" << axis << ">" << endl;
         return;
      }
   }

   // Output the position to the console
   cout << "NEW POSITION:" << endl;
   cout << new_pos << endl;

   robot.moveTo(new_pos);
}

/*
   Function       : convert_relative
   Description    : Returns (MINIMUM_AXIS_SETTING) + f * (MAXIMUM_AXIS_SETTING - MINIMUM_AXIS_SETTING)
   Arguments      : 
      - d         : A 'Direction' enum telling the function what axis it's dealing with.
      - f         : A number, between 0.0 and 1.0. 0.0 corresponds to the 
      - robot     : a reference to the Gantry RobotExt.
   Returns        : A float: see Description
*/
float convert_relative(Direction::Dir d, float f, RobotExt & robot)
{
   RobotPosition min = robot.currentLimits().min();
   RobotPosition max = robot.currentLimits().max();
   float min_amt, max_amt;
   float calculated;

   switch (d) {
      case Direction::X :  min_amt = min.x; max_amt = max.x; break;
      case Direction::Y :  min_amt = min.y; max_amt = max.y; break;
      case Direction::Z :  min_amt = min.z; max_amt = max.z; break;
      default:
         return 10e6; // break;
   }

   if (f < 0.0 || f > 1.0) return 10e6;


   calculated =  min_amt + (max_amt - min_amt) * f;
   return calculated;
}

/*
   Function       : hl_get_pos ('Position' Command Handler)
   Description    : Write the position of the Gantry RobotExt to standard output.
   Arguments      : 
      - arg_list  : a list of arguments to the function that will be ignored.
      - robot     : a reference to the Gantry RobotExt.
   Returns        : void
*/
void hl_get_pos(arg_list & args, RobotExt & robot)
{
   cout << "RobotExt Position: " << robot.currentPos() << endl;
}

/*
   Function       : hl_get_limits
   Description    : Display the limits of the Gantry RobotExt
   Arguments      : 
      - arg_list  : a list of arguments to the function that will be ignored.
      - robot     : a reference to the Gantry RobotExt.
   Returns        : void
*/
void hl_get_limits(arg_list & args, RobotExt & robot)
{
   cout << robot.currentLimits() << endl;
}



void hl_aaron_test(arg_list & args, RobotExt & robot)
{
   RobotPosition current_pos = robot.currentPos();
   RobotPosition entered_pos;

   cout << "Testing New Stuff" << endl;

   // DISPLAY ROBOT LIMITS TO COUT
   cout << "ROBOT LIMITS:" << endl;
   cout << robot.currentLimits() << endl;

   // DISPLAY CURRENT LIMITS TO COUT
   cout << "CURRENT POSITION:" << endl;
   cout << current_pos << endl;

///// CONSTRUCT A POSITION, check if it's within the limits
///double xpos, ypos, zpos;
///xpos = atof(args.front().c_str());
///args.pop_front();
///ypos = atof(args.front().c_str());
///args.pop_front();
///zpos = atof(args.front().c_str());
///args.pop_front();


   RobotPosition max = robot.currentLimits().max();
   RobotPosition min = robot.currentLimits().min();

   RobotPosition average_pos = max + min;
   average_pos.x /= 2.0;
   average_pos.y /= 2.0;
   average_pos.z /= 2.0;
   average_pos.j4 /= 2.0;
   average_pos.j5 /= 2.0;
   average_pos.j6 /= 2.0;

   // DISPLAY AVERAGE_POSITION TO COUT
   cout << "AVERAGE POSITION:" << endl;
   cout << average_pos << endl;

}

void hl_pos_smart(arg_list & args, RobotExt & robot);

/*
   Function       : construct_commands()
   Description    : Fill the commands 'vector'
   Arguments      : None
   Returns        : void
*/
void construct_commands() 
{
   Command c = {"Test", hl_help, "Test2"};
   // Initialize it
   commands = command_list();
   commands.push_back(Command({"Test", hl_help, "Test2"}));
   commands.push_back( {"help",  hl_help, "Display help." } );
   commands.push_back({ "?",     hl_help, "Display help." });
   commands.push_back({ "grid",  hl_move_xyz,   "Move to an x-y-z position." });
   commands.push_back({ "quit",  hl_quit, "Quit Program." });
   commands.push_back({ "pos",   hl_get_pos, "Get Position." });
   commands.push_back({ "lim",   hl_get_limits, "Get Limits." });

   // New shit
   commands.push_back({ "home", hl_home_smart, "Go Home." });

   commands.push_back({ "aaron", hl_aaron_test, "Test Stuff." });

   // Move to a set of X, Y, Z coordinates
   commands.push_back({ "3d", hl_move_to_position_float, "Move to Float." });
   commands.push_back({ "3df", hl_move_to_position_rel, "Move to Float." });

   commands.push_back({ "spos", hl_pos_smart, "Output position in table." });
}

/*
   Function       : execute_command
   Description    : Execute the correct command based on the string input from 'readline'
   Arguments      : 
      - command   : String containing text input from terminal
      - robot     : The Gantry RobotExt object
   Returns        : A reference to the string that was passed in, after trimming
*/
void execute_command(const string& command, RobotExt & robot)
{
   // Split the command into a list of arguments and a command name
   arg_list    args        = split(command);
   string      name        = args.front();
   args.pop_front();

   bool        found       = false; 
   int i = 0;

   // cout << "execute_command(): Name = <" << name << ">" << endl;

   // Search for a matching command
   command_iterator cmd;
   for (cmd = commands.begin(); cmd != commands.end(); cmd++)
   {
      if ((*cmd).name == name) {
         ((*cmd).func)(args, robot);
         found = true;
      }
   }

   if (!found) {
      cout << "Command <" << name << "> not found." << endl;
   }
}

/*
   Function       : trim
   Description    : Remove leading and trailing whitespace + tabs from a string.
   Arguments      : 
      - s         : The string to trim
   Returns        : A reference to the string that was passed in, after trimming
*/
string & trim (string & s)
{
   size_t pos = s.find_first_not_of(" \t");
   if (string::npos != pos)
   {
      s = s.substr(pos);
   }
   pos = s.find_last_not_of(" \t");
   if (string::npos != pos)
   {
      s = s.substr(0, pos+1);
   }
   return s;
}

/*
   Function       : Main
   Returns        : 0
*/
int main(int argc, char * argv[])
{
   // TODO assign rl_completion_entry_function
   // and get command autocomplete working

   // Make sure
   homed = false;


   construct_commands();

   // Initialize the robot
// RobotExt robot("/dev/gantry");
   RobotExt robot("/dev/cu.usbserial");
// robot.home(false); 
   while (!quit) {
      char * input = readline(Prompt.c_str());
      if (input) {
         cmdline = string(input);
         trim(cmdline);
         execute_command(cmdline, robot);

         // Free it, null it
         free(input);
         input = (char *) NULL;
      }
      else {
         cmdline = "exit";
         cout << "Exiting! (DEBUG: (!input) == TRUE)" << endl;
      }

   }

   return 0;
}


void hl_pos_smart(arg_list & args, RobotExt & robot)
{
   // get current position.
   const    RobotPosition current = robot.currentPos();
   const    RobotPosition max     = robot.currentLimits().max();
   const    RobotPosition min     = robot.currentLimits().min();
   
   // cout << sprintf("") << endl;


   printf("%-10s | %-10s | %-10s | %-10s\n", "AXIS", "MIN", "CURRENT", "MAX");
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "X", min.x, current.x, max.x);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Y", min.y, current.y, max.y);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "Z", min.z, current.z, max.z);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j4", min.j4, current.j4, max.j4);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j5", min.j5, current.j5, max.j5);
   printf("%-10s | %-+10.2f | %-+10.2f | %-+10.2f\n", "j6", min.j6, current.j6, max.j6);
   
}
