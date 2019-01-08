#include "Shell.h"

/*
   Name        :  Shell.cpp
   Author      :  Aaron
   Purpose     :  Implementation of the `Shell` class.
*/

Shell::Shell() :
   viewer(new pcl::visualization::PCLVisualizer("viewer")),
   _quit(false),
   _input_mutex(new Mutex())
{
   viewer->setBackgroundColor(0, 0, 0);
}

void Shell::register_function(callback call, std::string name, std::string description)
{
   struct shell_function func(call, name, description);
   functions.push_back(func);
}

void Shell::display_help()
{
   for (std::vector<shell_function>::const_iterator it = functions.begin();
         it != functions.end();
         it++)
   {
      std::cout << "<" << (*it).cmd << ">: " << (*it).description << std::endl;
   }
}

void Shell::get_input()
{
   _cmdline_input = readline("SHELL > ");
   _input = std::string(_cmdline_input);
   _input_mutex->lock();
   _received_input = true;
   _input_mutex->unlock();
}

void Shell::run_shell()
{
   while (!_quit) {

      boost::thread     input_thread(boost::bind(&Shell::get_input, this));

      // IMPORTANT: ANYTHING INVOLVING THE VIEWER NEEDS TO BE IN THE MAIN THREAD.
      // __ANYTHING__ ELSE CAN GO IN A SEPARATE THREAD.
      // DO NOT DO ANYTHING LIKE THIS OUTSIDE OF THE MAIN THREAD!
      while (!viewer->wasStopped())
      {
         // Let the user interact with the viewer for a bit
         viewer->spinOnce(100);
         boost::this_thread::sleep(boost::posix_time::microseconds(100000));

         Mutex::ScopedLock(*_input_mutex);
         if (_received_input) {
            break;
         }
      }

      input_thread.join();
      _received_input = false;


      if (stringmanip::trim(_input).length() != 0)
      {
         _arguments     = stringmanip::split(_input);
         _cmd_name      = _arguments.front();
         _arguments.pop_front();

         bool function_found = false;

         if (_input == "quit") {
            _quit = true;
            break;
         }


         for (std::vector<shell_function>::const_iterator it = functions.begin();
               it != functions.end();
               it++)
         {
            if( (*it).cmd == _cmd_name) {
               // execute the function
               (*it).function(_arguments);
               free(_cmdline_input);
               function_found = true;
               break;
            }
         }

         if (!function_found) {
            cout << "Command not recognized." << endl;
            display_help();
         }
      }


   }

   free(_cmdline_input);
}
