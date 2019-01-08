#include "WSObject.h"

WSObject::WSObject(float x, float y, float obs_dist) :
   x_position(x), y_position(y), observation_distance(obs_dist)
{
   id = current_id++;

   r_display = 0;
   g_display = 0;
   b_display = 0;
}

// Copy constructor
WSObject::WSObject(const WSObject & copy) {

   x_position = copy.x_position;
   y_position = copy.y_position;

   observation_distance = copy.observation_distance;
   id         = copy.id;
   x_obs_position = copy.x_obs_position;
   y_obs_position = copy.y_obs_position;

   r_display = copy.r_display;
   g_display = copy.g_display;
   b_display = copy.b_display;
           plane_distance = copy.plane_distance;
}

bool WSObject::operator== (const WSObject& ws) const {
    return id == ws.id &&
           x_position == ws.x_position &&
           y_position == ws.y_position &&
           x_obs_position == ws.x_obs_position &&
           y_obs_position == ws.y_obs_position &&
           observation_distance == ws.observation_distance;
           plane_distance == ws.plane_distance;
}

// Static initiations
int WSObject::current_id = 0;
float WSObject::fudge = 60.0; // TODO FIND A WAY TO LOWER THIS
float WSObject::distance_fudge = 300; // RobotPosition radius of 500.
