#include <iostream>
#include <QtWidgets/QApplication>
#include "mainwindow.h"
//#include "pclviewer2.h"
//#include "calibrationwindow.h"
//#include "itemdisplaywindow.h"
#include "gantrywindow.h"

using namespace std;

int main(int argc, char *argv[])
{
    //cout << "Hello World!" << endl;

    QApplication app(argc, argv);
//  PCLViewer2 pv;
//  pv.show();
//  CalibrationWindow cw;
//  cw.show();
//  ItemDisplayWindow iw;
//  iw.show();
    GantryWindow gw;
    gw.show();


    return app.exec();
}
