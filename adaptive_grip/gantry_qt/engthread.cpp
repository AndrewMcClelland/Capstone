#include "engthread.h"

EngThread::EngThread(Engine2 * _eng, const RobotPosition & _pos) :
   QThread()
{
   eng = _eng;
   pos = _pos;
}
void EngThread::run()
{
   std::cout << "EngThread::run() beginning.." << std::endl;
   eng->moveTo(pos);
   std::cout << "EngThread::run() ending.." << std::endl;
}
void EngThread::setPos(const RobotPosition & _pos)
{
   pos = _pos;
}
