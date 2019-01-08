#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <QMainWindow>

#include <string>

#include <iostream>
#include "Robot.h"

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT

	typedef enum {
        CALIBRATE_Z_AXIS_UP,
        CALIBRATE_Z_AXIS_DOWN,
        CALIBRATE_Y_AXIS_UP,
        CALIBRATE_Y_AXIS_DOWN,
        CALIBRATE_X_AXIS_UP,
        CALIBRATE_X_AXIS_DOWN,
		DONE
	} State;

	State	state;

	Robot * robot;
	bool home_ready;
	bool home_done;
    
public:
    explicit CalibrationWindow(QWidget *parent = 0);
    ~CalibrationWindow();

	 void appendAndRun(std::string);
	 void append(std::string);
    
private:
    Ui::CalibrationWindow *ui;

    void home_sequence();
    bool waitForEnter();
	 void waitThenRun(std::string s);

public slots:
	void calibrationButtonPressed();
	void finishedButtonPressed();
    void homeButtonPressed();

};

#endif // CALIBRATIONWINDOW_H
