#ifndef COLOURS__H
#define COLOURS__H

/*
   Name        :  colours.h
   Author      :  Aaron
   Purpose     :  Global namespace with useful structs and definitions
                  to easily access some predefined colours.
*/

#define NUM_COLOURS 4

namespace colours {

   struct colour_rgb {

      // No argument constructor
      colour_rgb () {}

      // RGB constructor
      colour_rgb (unsigned int red_,
                  unsigned int green_,
                  unsigned int blue_) :
         red(red_), green(green_), blue(blue_) {}

      unsigned int red;
      unsigned int green;
      unsigned int blue;
   };

   typedef struct colour_rgb colour_rgb_t;

   const int RED = 0;
   const int GREEN = 1;
   const int BLUE = 2;
   const int PURPLE = 3;

   const colour_rgb_t colours[] = {
      colour_rgb_t(255, 0, 0), // red
      colour_rgb_t(0, 255, 0), // green
      colour_rgb_t(0, 191, 255), // blue
      colour_rgb_t(153, 0, 255) // purple
   };


} // namespace

#endif // COLOURS__H
