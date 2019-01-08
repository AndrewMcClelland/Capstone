#ifndef LOGGER__H
#define LOGGER__H

#include <iostream>
#include <string>
#include <sstream>

/*
   Name        :  Logger.h
   Author      :  Aaron
   Purpose     :  A Logger class to record useful information.
                  This is a base class that sends everything to cout.
                  You should extend this class if you want to log differently
                  (formatted output, log to file, etc.)
*/

class Logger {

public:

   // Default Constructor
   Logger();

   // Base "log" method: print to standard output
   virtual void log(std::string str);

   // Same as above but with a stringstream
   virtual void log(std::stringstream &ss);

};

#endif // LOGGER__H
