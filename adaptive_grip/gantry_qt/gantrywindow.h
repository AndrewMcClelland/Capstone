#ifndef GANTRYWINDOW_H
#define GANTRYWINDOW_H

#include <QtWidgets/QMainWindow>

#include "Engine2.h"
#include "Engine3.h"
#include "stringmanip.h"
#include "inst/pcl_visualizer.h"
#include "QLogger.h"
#include "RobotPosition.h"
#include "Clawduino.h"
#include "engthread.h"
#include "liveviewer.h"
//#include "liveupdatethread.h"

#include <vtkRenderWindow.h>
#define BUTTON_SPEED_NOT_Z_AXIS 0.6
#define BUTTON_SPEED_Z_AXIS 0.2

// TODO move this somewhere better
// can't do Q_ASSERT(CONNECT( ... )) b/c assert expands to a no-op in release builds
#define CHECKED_CONNECT(source, signal, receiver, slot) \
   if (!connect(source, signal, receiver, slot)) \
      qt_assert_x(Q_FUNC_INFO, "CHECKED_CONNECT failed", __FILE__, __LINE__);

namespace Ui {
class GantryWindow;
}

class GantryWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GantryWindow(QWidget *parent = 0);
    ~GantryWindow();

    EngThread * et;
    Logger *      m_logger;

    pcl::visualization::PCLVisualizer::Ptr       viewer;
    Engine2 *  eng;

//  pcl::visualization::PCLVisualizer::Ptr       live_viewer;
    LiveViewer * live_viewer;

 	// TODO FOR NOW TODO 
	Clawduino * clawduino;
 	// TODO FOR NOW TODO 

    float      current_x_cal, current_y_cal; // TODO move

private:
    Ui::GantryWindow *ui;

public slots:

 	void update_display();
   void home();
   void load_calibration();
   void move_to_location();
   void find_objects();
   void update_position();
   void center_item();
   void changeXCal(double);
   void changeYCal(double);
   void clear_objects();
   void manualMove(int);
   void sendSerial();
	void startLiveViewerFeed();
	void stopLiveViewerFeed();
};

#endif // GANTRYWINDOW_H
