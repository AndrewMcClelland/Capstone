#ifndef COMMON_DEFS__H
#define COMMON_DEFS__H

/*
   Name        :  common_defs.h
   Author      :  Aaron
   Purpose     :  A place to drop some common macros
*/

// (int) return codes
#define SUCCESS 0
#define FAILURE 1

// Points 0.05 (5cm) above the floor and closer are removed, by default.
#define DEFAULT_FLOOR_HEIGHT 0.10 // Better than 0.05. Gets rid of wood on the floor too.

// Empirically tested - for a max height of 0.10 + 0.25 = 0.35m
#define DEFAULT_PLANE_SEPARATION 0.25

#define PI 3.14159265

// How far away, from the centroid of an object, in the X direction,
// that we should be to get the best view of it.
#define VANTAGE_X_OFFSET 400.0

// How far away, from the centroid of an object, in the Y direction,
// that we should be to get the best view of it.
#define VANTAGE_Y_OFFSET 0.0

#endif
