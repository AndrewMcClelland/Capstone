#include "Calibration.h"

bool Calibration::toFile(const std::string& filename)
{
   using namespace std;
   ofstream outfile(filename.c_str());
   if (outfile) {
      outfile << x_rpos_amt << " " << y_rpos_amt << " "
         << x_vector.x << " " << x_vector.y << " " << x_vector.z << " "
         << y_vector.x << " " << y_vector.y << " " << y_vector.z << std::endl;
      return true;
   } else return false;
}

bool Calibration::fromFile(const std::string& filename)
{
   using namespace std;
   ifstream infile(filename.c_str());
   float x_vec_x, x_vec_y, x_vec_z;
   float y_vec_x, y_vec_y, y_vec_z;
   if (infile) {
      infile >> x_rpos_amt >> y_rpos_amt
         >> x_vec_x >> x_vec_y >> x_vec_z 
         >> y_vec_x >> y_vec_y >> y_vec_z;
      x_vector.x = x_vec_x;
      x_vector.y = x_vec_y;
      x_vector.z = x_vec_z;
      y_vector.x = y_vec_x;
      y_vector.y = y_vec_y;
      y_vector.z = y_vec_z;

      {
         x_adj_amt = -2.80;
         y_adj_amt = 0.8;
         cerr << "Calibration::fromFile() WARNING - Initializing x_adj_vector to: " << x_adj_amt << endl;;
         cerr << "Calibration::fromFile() WARNING - Initializing y_adj_vector to: " << y_adj_amt << endl;;
      }

      return true;
   } else return false;
}


std::string Calibration::toString()
{
   std::stringstream ss;
   ss << x_rpos_amt << " " << y_rpos_amt << " "
      << x_vector.x << " " << x_vector.y << " "<< x_vector.z << " "
      << y_vector.x << " " << y_vector.y << " "<< y_vector.z << std::endl;
   return ss.str();
}
