#ifndef ENGTHREAD__H
#define ENGTHREAD__H

//#include <QtGui>
#include "Engine2.h"
#include <iostream>
#include <QtCore/QThread>

class EngThread : public QThread
{
   Q_OBJECT

      Engine2 * eng; 
      RobotPosition pos;
   public:

      EngThread(Engine2 * _eng, const RobotPosition & _pos);
      void setPos(const RobotPosition & _pos);

   private:

      void run();
};

/*
class UpdateCamThread : public QThread
{
      Q_OBJECT
      pcl::visualization::PCLVisualizer::Ptr    vis;
}
*/


#endif
