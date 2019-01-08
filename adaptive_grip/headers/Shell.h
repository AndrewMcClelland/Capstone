#ifndef SHELL__H
#define SHELL__H

/*
   Name        :  Shell.h
   Author      :  Aaron
   Purpose     :  A class to easily create a shell that can be used to
                  test new features.
                  See programs/test_two_plane_filter.cpp for an example of how to 
                  use this class.
*/

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include "stringmanip.h"
#include "Mutex.h"
#include <string>
#include <vector>
#include <iostream>
#include <inst/pcl_visualizer.h> // Visualizing

// Convenience macro.
#define SHELL_ADD_FUNCTION(SHELL, FUNCTION, NAME, DESC) \
{ \
   Shell::callback SHELL_CALLBACK_FUNCTION = boost::bind(&FUNCTION, _1); \
   SHELL.register_function(SHELL_CALLBACK_FUNCTION, NAME, DESC); \
}

extern "C" {
   #include "readline/readline.h"
}
using stringmanip::arg_list;


class Shell {


   public: 

   pcl::visualization::PCLVisualizer::Ptr    viewer;

   typedef boost::function<void (const arg_list &)>   callback;

   struct shell_function {
      callback                                       function;
      std::string                                    cmd;
      std::string                                    description;

      shell_function(callback _function, std::string _cmd, std::string _description) :
         function(_function), cmd(_cmd), description(_description) {}
   };


   // Default Constructor.
   Shell();

/*
   Function       : run_shell
   Description    : Begins the shell's operation.
                    The shell will accept a line of input from standard input,
                    look for a registered function of the same name, and execute
                    it if it exists. Additional arguments will be packed into an
                    `arg_list` (std::list<string>) and sent to the registered
                    function.
                    Entering `quit` with or without arguments will stop the shell.
*/
   void run_shell(); // The "main" method.

/*
   Function       : register_function
   Description    : Registers a function with the shell.
   Arguments      : 
      call        : A boost::fuction<void (const arg_list &)>, i.e. a function
                    that returns `void` and takes a const arg_list as input.
                    This function is the function that is executed when the 
                    appropriate command is entered into the shell.

                    the `bind_functions()` function in programs/test_two_plane_filter.cpp
                    shows how to construct boost::functions.
                    (There really isn't an easier way -- C++ function pointers without using the
                    boost library have notoriously hard-to-use syntax)

      name        : The name of the command that is entered into the shell to 
                    execute the above function.
      description : A short description of the function that will be displayed
                    in a help message.
*/
   void register_function(callback call, std::string name, std::string description);

/*
   Function       : register_function
   Description    : Displays all possible commands and their descriptions.
*/
   void display_help();



   private:

   void get_input();
         
   std::vector<shell_function>   functions;
   bool                          _received_input;
   Mutex *                       _input_mutex;
   std::string                   _input;

   arg_list                      _arguments;
   std::string                   _cmd_name;
   bool                          _quit;
   char *                        _cmdline_input;
   


};

#endif // SHELL__H
