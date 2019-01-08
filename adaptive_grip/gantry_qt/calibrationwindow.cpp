#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"


using namespace std;

CalibrationWindow::CalibrationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CalibrationWindow)
{
    ui->setupUi(this);

	 robot = new Robot("/dev/gantry");

	 home_ready = false;
	 home_done = false;

    state = CALIBRATE_X_AXIS_UP;

    std::string message;
    message += "Beginning +X axis calibration.";

    ui->textEdit->setText(QString("Beginning +X axis calibration."));

    QObject::connect(ui->calButton, SIGNAL(pressed()), this, SLOT(calibrationButtonPressed()));
    QObject::connect(ui->finishedButton, SIGNAL(pressed()), this, SLOT(finishedButtonPressed()));
    QObject::connect(ui->homeButton, SIGNAL(pressed()), this, SLOT(homeButtonPressed()));

}

CalibrationWindow::~CalibrationWindow()
{
    delete ui;
}

void 
CalibrationWindow::appendAndRun(std::string command)
{
	append("Running: <" + command + ">");
	robot->runCmd(command);
}

void 
CalibrationWindow::append(std::string command)
{
    ui->textEdit->append(QString::fromStdString(command));
}

void
CalibrationWindow::calibrationButtonPressed()
{
	if (state == CALIBRATE_Z_AXIS_UP) 
	{
        appendAndRun("JOINT 3, 50");
    } else if (state == CALIBRATE_Y_AXIS_UP) {
        appendAndRun("JOINT 2, 50");
    } else if (state == CALIBRATE_X_AXIS_UP) {
        appendAndRun("JOINT 1, 50");
    } else if (state == CALIBRATE_Z_AXIS_DOWN) {
        appendAndRun("JOINT 3, -50");
    } else if (state == CALIBRATE_Y_AXIS_DOWN) {
        appendAndRun("JOINT 2, -50");
    } else if (state == CALIBRATE_X_AXIS_DOWN) {
        appendAndRun("JOINT 1, -50");
	} else {
		append("Nothing left to calibrate.");
	}

}

        void
CalibrationWindow::finishedButtonPressed()
{
	if (state == CALIBRATE_X_AXIS_UP) {
		append("+X-axis positioning finished. Beginning -X-axis positioning.");
        state = CALIBRATE_X_AXIS_DOWN;
	} else if (state == CALIBRATE_X_AXIS_DOWN) {
		append("-X-axis positioning finished. Beginning +Y-axis positioning.");
        state = CALIBRATE_Y_AXIS_UP;
	} else if (state == CALIBRATE_Y_AXIS_UP) {
		append("+Y-axis positioning finished. Beginning -Y-axis positioning.");
        state = CALIBRATE_Y_AXIS_DOWN;
	} else if (state == CALIBRATE_Y_AXIS_DOWN) {
		append("-Y-axis positioning finished. Beginning +Z-axis positioning.");
        state = CALIBRATE_Z_AXIS_UP;
	} else if (state == CALIBRATE_Z_AXIS_UP) {
		append("+Z-axis positioning finished. Beginning -Z-axis positioning.");
        state = CALIBRATE_Z_AXIS_DOWN;
	} else if (state == CALIBRATE_Z_AXIS_DOWN) {
		append("-Z-axis positioning finished.");
        state = DONE;
	} else {
        append("Calibration is finished.");
	}
}

void
CalibrationWindow::homeButtonPressed()
{
    if (state != DONE) {
        append("You need to finish calibrating X/Y/Z before executing the home sequence.");
    } else {
        home_sequence();
    }
}

bool
CalibrationWindow::waitForEnter()
{
	const char Enter = 0x0a;
	char c = 0;
	do {
		cin.get(c);
	} while (c != Enter);
	return true;
}

void
CalibrationWindow::waitThenRun(std::string s)
{
	cout << "Press Enter to run: [" << s << "]" << endl;
	waitForEnter();
	appendAndRun(s);
}


void
CalibrationWindow::home_sequence()
{
	if (!home_ready) {
		append("PLEASE MAKE SURE YOU ARE HOLDING THE EMERGENCY STOP ON THE TEACH PENDANT.");
		append("PRESS THE HOME BUTTON AGAIN ONCE YOU ARE READY.");
		append("YOU HAVE BEEN WARNED.");
		home_ready = true;
	} else {
		if (!home_done) {
			
			waitThenRun("SPEED 50");
			waitThenRun("JOINT 2, -200");
			waitThenRun("JOINT 2, -200");
			waitThenRun("JOINT 2, -200");

	// Disecting HOMEAX 1, 10000..
	/*
		ENABLE SLEW
		SPEED 10
		MOTOR [0] [1], OFFHOME
		HOMESEQ [0]
		DISABLE SLEW
	*/
		waitThenRun("ENABLE SLEW");
		waitThenRun("SPEED 10");
		waitThenRun("MOTOR 1,10000,OFFHOME");
		waitThenRun("HOMESEQ 1");
		waitThenRun("DISABLE SLEW");

		// Then, move in the +x direction so we can home the Y axis.
		// Done to keep the claw from hitting corners.

		waitThenRun("SPEED 50");
		waitThenRun("JOINT 1, 200");
		waitThenRun("JOINT 1, 200");
		waitThenRun("JOINT 1, 200");

		// Similarly, this is the dissected HOMEAX 2, -1000
		waitThenRun("ENABLE SLEW");
		waitThenRun("SPEED 10");
		waitThenRun("MOTOR 2,-1000,OFFHOME");
		waitThenRun("HOMESEQ 2");
		waitThenRun("DISABLE SLEW");

		// Finish up with the Z axis.. Move in -Y so we don't hit the corner when we go down.
		waitThenRun("SPEED 50"); 
		waitThenRun("JOINT 2, -200");
		waitThenRun("JOINT 2, -200");
		waitThenRun("JOINT 2, -200");

		waitThenRun("ENABLE SLEW");
		waitThenRun("SPEED 10");
		waitThenRun("MOTOR 3,-1000,OFFHOME");
		waitThenRun("HOMESEQ 3");
		waitThenRun("DISABLE SLEW");

		// And finally home the lower joints.
		waitThenRun("HOMESEQ 4");
		waitThenRun("HOMESEQ 5");
		waitThenRun("HOMESEQ 6");

		home_done = true;

		} else {
			append("The home sequence has already been executed.");
		}
	}
}
