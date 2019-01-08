#include "gantrywindow.h"
#include "ui_gantrywindow.h"

//#define DEBUG_LIVE_VIEWER

using std::cout;
using std::endl;
using pcl::OpenNIGrabber;

const int DECREASE_X = 1;
const int DECREASE_Y = 2;
const int DECREASE_Z = 3;
const int DECREASE_J4 = 4;
const int DECREASE_J5 = 5;
const int DECREASE_J6 = 6;
const int INCREASE_X = 101;
const int INCREASE_Y = 102;
const int INCREASE_Z = 103;
const int INCREASE_J4 = 104;
const int INCREASE_J5 = 105;
const int INCREASE_J6 = 106;

//#define NO_ENGINE

GantryWindow::GantryWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GantryWindow),
    viewer(new pcl::visualization::PCLVisualizer("Viewer", false)) // false = no interactor
{

    ui->setupUi(this);

    m_logger = new QLogger(ui->statusDisplay);
//eng = new Engine3(viewer, m_logger);

//#ifndef NO_ENGINE
	eng = new Engine2(viewer, m_logger);
   ui->itemDisplay->m_objects = eng->m_objects;
//#endif


   live_viewer = new LiveViewer(ui->qvtkWidget, viewer, eng->vp_calibration_axes);

    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();



    // CHECKED_CONNECTs
    CHECKED_CONNECT(ui->homeButton, SIGNAL(pressed()), this, SLOT(home()));
    CHECKED_CONNECT(ui->loadCalButton, SIGNAL(pressed()), this, SLOT(load_calibration()));
    CHECKED_CONNECT(ui->moveButton, SIGNAL(pressed()), this, SLOT(move_to_location()));
    CHECKED_CONNECT(ui->scanButton, SIGNAL(pressed()), this, SLOT(find_objects()));
    CHECKED_CONNECT(ui->positionButton, SIGNAL(pressed()), this, SLOT(update_display()));
    CHECKED_CONNECT(ui->prevItemButton, SIGNAL(pressed()), ui->itemDisplay, SLOT(cycle()));
    CHECKED_CONNECT(ui->nextItemButton, SIGNAL(pressed()), ui->itemDisplay, SLOT(cycle()));
    CHECKED_CONNECT(ui->centerItemButton, SIGNAL(pressed()), this, SLOT(center_item()));
    CHECKED_CONNECT(ui->cal_x_spin, SIGNAL(valueChanged(double)), this, SLOT(changeXCal(double)));
    CHECKED_CONNECT(ui->cal_y_spin, SIGNAL(valueChanged(double)), this, SLOT(changeYCal(double)));
    CHECKED_CONNECT(ui->clearObjectsButton, SIGNAL(pressed()), this, SLOT(clear_objects()));
//  CHECKED_CONNECT(ui->startLiveButton, SIGNAL(pressed()), this, SLOT(startLiveFeed()));
//  CHECKED_CONNECT(ui->stopLiveButton, SIGNAL(pressed()), this, SLOT(stopLiveFeed()));

    CHECKED_CONNECT(ui->arduinoButton, SIGNAL(pressed()), this, SLOT(sendSerial()));

    ui->cal_y_spin->setMinimum(-100.0);
    ui->cal_x_spin->setMinimum(-100.0);

	 // New for Live Viewer...
	 CHECKED_CONNECT(eng, SIGNAL(RestartLiveFeed()), this, SLOT(startLiveViewerFeed()));
	 CHECKED_CONNECT(eng, SIGNAL(StopLiveFeed()), this, SLOT(stopLiveViewerFeed()));
	
  
    /*
    // Signal-mapping the slots for the "move" "here" buttons
    QSignalMapper * buttonMapper = new QSignalMapper(this);
    buttonMapper->setMapping(ui->decreaseXButton, DECREASE_X);
    buttonMapper->setMapping(ui->decreaseYButton, DECREASE_Y);
    buttonMapper->setMapping(ui->decreaseZButton, DECREASE_Z);
    buttonMapper->setMapping(ui->decreasej4Button, DECREASE_J4);
    buttonMapper->setMapping(ui->decreasej5Button, DECREASE_J5);
    buttonMapper->setMapping(ui->decreasej6Button, DECREASE_J6);
    buttonMapper->setMapping(ui->increaseXButton, INCREASE_X);
    buttonMapper->setMapping(ui->increaseYButton, INCREASE_Y);
    buttonMapper->setMapping(ui->increaseZButton, INCREASE_Z);
    buttonMapper->setMapping(ui->increasej4Button, INCREASE_J4);
    buttonMapper->setMapping(ui->increasej5Button, INCREASE_J5);
    buttonMapper->setMapping(ui->increasej6Button, INCREASE_J6);
    CHECKED_CONNECT(ui->decreaseXButton, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->decreaseYButton, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->decreaseZButton, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->decreasej4Button, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->decreasej5Button, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->decreasej6Button, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->increaseXButton, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->increaseYButton, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->increaseZButton, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->increasej4Button, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->increasej5Button, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    CHECKED_CONNECT(ui->increasej6Button, SIGNAL(pressed()), buttonMapper, SLOT(map()));
    QObject::connect(buttonMapper, SIGNAL(mapped(int)), this, SLOT(manualMove(int)));
    */
        
    // Thread
   // et = new EngThread(eng, eng->m_robot->currentPos());
   // QObject::connect(et, SIGNAL(finished()), et, SLOT(quit()));

	 // Try to initialize an Arduino connected to /dev/ttyUSB1
	 clawduino = new Clawduino(m_logger, "/dev/ttyUSB1");

	 // Set a default "claw line"
////#define DEFAULT_CLAW_OFFSET -0.75, -3.15
	 m_logger->log("Setting default 'claw' offset to -0.75, -3.15.");

//#ifndef NO_ENGINE
    eng->move_claw_line(-0.75, -3.15, 0);
//	ui->chartView->updateSeries();
//#endif

}

//void GantryWindow::startLiveFeed() { 
//   live_viewer->start();
//}
//void GantryWindow::stopLiveFeed() {
//    live_viewer->stop();
//}


void
GantryWindow::home()
{
    // Ryan changed HOME5. It's safe.
    std::string s = "RUN HOME5";
    eng->m_robot->runCmd(s);
    update_display();
}

void 
GantryWindow::center_item()
{
   eng->move_to_object(ui->itemDisplay->get_index());
   update_display();

     ui->chartView->surface = eng->surface;
	ui->chartView->updateSeries();
}

void 
GantryWindow::load_calibration()
{
   eng->load();
// RobotPosition new_pos = eng->m_robot->currentPos();
   update_display();


//  current_x_cal = -3.05;
//  current_y_cal = -0.95;
    current_x_cal = -3.95;
    current_y_cal = -0.55;
   eng->move_claw_line(current_x_cal, current_y_cal, 0);
}


void
GantryWindow::move_to_location()
{
// QString  editor_text = ui->positionInput->toPlainText();
   std::string editor_text = ui->positionInput->toPlainText().toUtf8().constData();

   RobotPosition new_pos = eng->m_robot->currentPos();

   using stringmanip::arg_list;
   arg_list args = stringmanip::split(editor_text);

   bool z_set = false;

   while (!args.empty()) {
      std::string axis = args.front();
      float value;
      args.pop_front();
      if (args.empty()) {
         cout << "Illegal command." << endl;
         return;
      } else {
         value = atof(args.front().c_str());
         args.pop_front();
      }

      if      (axis == "x")      new_pos.x = value;
      else if (axis == "y")      new_pos.y = value;
      else if (axis == "z")      {
          new_pos.z = value;
          z_set = true;
      }
      else if (axis == "j4")     new_pos.j4 = value;
      else if (axis == "j5")     new_pos.j5 = value;
      else if (axis == "j6")     new_pos.j6 = value;
      else {
         cout << "Unrecognized argument: <" << axis << ">" << endl;
         return;
      }
   }

   if (!z_set) new_pos.z = 200;

   std::cout << "New Position: " << std::endl;
   std::cout << new_pos << std::endl;

// eng->m_robot->moveTo(new_pos);
  
// Need to be updating the other view in another thread
if (z_set) {
	 eng->moveTo(new_pos, true);
 } else {
	 eng->moveTo(new_pos, true, XY_SPEED);
 }

  // et->setPos(new_pos);
  // et->start();

// std::string reply;
// eng->m_robot->controller >> reply;

// std::cout << "REPLY: " << reply << endl;
// 
// update_position();
   update_display();
}

void GantryWindow::find_objects()
{
// eng->locate(true);
   eng->scan();
   update_display();

   // Trigger a paint event?
}

void GantryWindow::update_position()
{
   const RobotPosition curr = eng->getPosition();

   ui->x_pte->setPlainText(QString("x : ") + QString::number(curr.x));
   ui->y_pte->setPlainText(QString("y : ") + QString::number(curr.y));
   ui->z_pte->setPlainText(QString("z : ") + QString::number(curr.z));
   ui->j4_pte->setPlainText(QString("J4: ") + QString::number(curr.j4));
   ui->j5_pte->setPlainText(QString("J5: ") + QString::number(curr.j5));
   ui->j6_pte->setPlainText(QString("J6: ") + QString::number(curr.j6));

   ui->itemDisplay->update_robot_position(curr);
// update_display();
}

void GantryWindow::update_display()
{
	update_position();
   ui->itemDisplay->update();
}

GantryWindow::~GantryWindow()
{
    delete ui;
}

void GantryWindow::changeXCal(double d) {
   current_x_cal = d;
   cout << "current_x_cal = " << current_x_cal << endl;
   eng->move_claw_line(current_x_cal, current_y_cal, 0);
   clear_objects();
}

void GantryWindow::changeYCal(double d) {
   current_y_cal = d;
   cout << "current_y_cal = " << current_y_cal << endl;
   eng->move_claw_line(current_x_cal, current_y_cal, 0);
   clear_objects();
}

void GantryWindow::clear_objects()
{
   eng->clear_objects();
   update_display();
}

void GantryWindow::manualMove(int direction)
{
   // degrees: X01, X01, X12
   // angles: X10, X11, X12
   // increase: 0XX
   // decrease: 1XX
   bool decreasing = (direction >= 100);
   bool angles     = ((direction % 100) > 3);

	std::stringstream msg;
	msg << "GantryWindow::manualMove() - direction = " << direction;
	m_logger->log(msg);

	const float degree_mm = 50.0;
	const float angle_deg = 10.0;

	float increment;
	if (angles) {
		increment = angle_deg;
	} else {
		increment = degree_mm;
	}

	if (decreasing) {
		increment = increment * -1.0;
	} 

	RobotPosition newPosition = eng->getPosition();

   if (direction % 100 == 1) {
		newPosition.x += increment;
   } else if (direction % 100 == 2) {
		newPosition.y += increment;
   } else if (direction % 100 == 3) {
		newPosition.z += increment;
   } else if (direction % 100 == 4) {
		newPosition.j4 += increment;
   } else if (direction % 100 == 5) {
		newPosition.j5 += increment;
   } else if (direction % 100 == 6) {
		newPosition.j6 += increment;
   }

	{
		stringstream msg;
		msg << "GantryWindow::manualMove - would move to the position: " << endl;
		msg << newPosition << endl;
		msg << "Add the moveTo() to finish implementation.";
		m_logger->log(msg);
	}

	float speed;
	if (direction % 100 == 4) // Z-axis
	{
	 	speed = BUTTON_SPEED_Z_AXIS;
	} else {
		speed = BUTTON_SPEED_NOT_Z_AXIS;
	}

	eng->moveTo(newPosition, true, speed);
   update_display();
} 
void GantryWindow::sendSerial()
{
	string output;
    std::string text_to_send = ui->arduinoInput->toPlainText().toUtf8().constData();

	int return_code = clawduino->send_message(text_to_send, output);

	if (return_code == SUCCESS) {
//////std::stringstream response;
//////response << "GantryWindow::sendSerial() - got response [" << output << "]";
//////m_logger->log(response);
		m_logger->log("GantryWindow::sendSerial() - successful.");
	} else {
		m_logger->log("GantryWindow::sendSerial() - could not send message.");
	}
}

void GantryWindow::startLiveViewerFeed()
{
	live_viewer->start();
}

void GantryWindow::stopLiveViewerFeed()
{
	live_viewer->stop();
}
