#include "stringmanip.h"

/*
   Function       : trim
   Description    : Remove leading and trailing whitespace + tabs from a std::string.
   Arguments      : 
      - s         : The std::string to trim
   Returns        : A reference to the std::string that was passed in, after trimming
*/
std::string & stringmanip::trim (std::string & s)
{
   size_t pos = s.find_first_not_of(" \t");
   if (std::string::npos != pos)
   {
      s = s.substr(pos);
   }
   pos = s.find_last_not_of(" \t");
   if (std::string::npos != pos)
   {
      s = s.substr(0, pos+1);
   }
   return s;
}

/*
   Function       : split 
   Arguments      : 
      s: a string, with one or more words separated by spaces.
   Returns        : each space-delimited word in 's' is an entry
                    in the returned list.

*/
stringmanip::arg_list stringmanip::split (const std::string& s) 
{
   istringstream line(s);
   std::string word;
   std::list<std::string> words;
   while (line >> word) words.push_back(word);
   return words;
}
