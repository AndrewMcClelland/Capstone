#include "liveupdatethread.h"

LiveUpdateThread::LiveUpdateThread(LiveViewer * _lv, QVTKWidget *_qv) :
   QThread(),
   lv(_lv),
   qv(_qv)
{
}

void LiveUpdateThread::run()
{
   int count = 0;
   lv->start();

   while (count < 100)
   {
      lv->mutex->lock();
      qv->update();
      lv->mutex->unlock();
   }
   lv->stop();
   std::cout << "Finished LiveUpdateThread::run()" << std::endl;
}
