#ifndef  STRINGMANIP__H
#define  STRINGMANIP__H

/*
   Name        :  stringmanip.h
   Author      :  Aaron
   Purpose     :  A global namespace containing some utility functions
                  that are used for string manipulation.
*/

#include <list>
#include <string>
#include <sstream>

namespace stringmanip {

   using std::istringstream;
   using std::list;
   using std::string;

   typedef std::list<std::string>   arg_list;

   
   /*
      Function       : trim
      Description    : Remove leading and trailing whitespace from a string.
      Arguments      : s - the string to operate on.
      Returns        : A reference to s, with leading and trailing whitespace removed.
   */
   std::string &                    trim (std::string & s);

   /*
      Function       : split
      Description    : Split a string, using whitespaces as delimiters, into a list of 
                       strings (typedeffed as `arg_list`)
      Arguments      : s - the string to operate on.
      Returns        : A list of strings (`arg_list`). Each entry in the list is a word
                       from the original string s.
   */
   arg_list                         split (const std::string& s);

}

#endif   // STRINGMANIP__H
