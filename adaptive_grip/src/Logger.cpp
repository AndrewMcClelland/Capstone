#include "Logger.h"

Logger::Logger()
{}

void 
Logger::log(std::string str)
{
   std::cout << str << std::endl;
}

void 
Logger::log(std::stringstream &ss){
// log(ss.str());
   std::cout << ss.str() << std::endl;
}
