#include "pclviewer.h"
#include "ui_pclviewer.h"

PCLViewer::PCLViewer(QWidget *parent = NULL) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer)
{
    ui->setupUi(this);
}

PCLViewer::~PCLViewer()
{
    delete ui;
}
