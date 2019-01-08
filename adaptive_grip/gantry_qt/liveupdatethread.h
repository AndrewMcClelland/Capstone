#ifndef LIVEUPDATETHREAD__H
#define LIVEUPDATETHREAD__H

//#include <QtGui>
#include <QVTKWidget.h>
#include "liveviewer.h"

class LiveUpdateThread : public QThread
{
   Q_OBJECT
   public:

   LiveViewer * lv;
   QVTKWidget * qv;

   LiveUpdateThread(LiveViewer *, QVTKWidget *);
   void run();
};

#endif
