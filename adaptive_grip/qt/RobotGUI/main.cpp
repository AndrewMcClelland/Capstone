#include "RobotGUI.h"

#include <QtWidgets>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    RobotGUI * gui = new RobotGUI();

    return a.exec();
}
