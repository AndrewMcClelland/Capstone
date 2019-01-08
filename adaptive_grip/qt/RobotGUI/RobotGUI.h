#ifndef ROBOTGUI_H
#define ROBOTGUI_H

#include "RobotLimits.h"
#include "ItemsImage.h"
#include "../../headers/WSObject.h"

#include <QWidget>
#include <QList>
#include <QGraphicsScene>
#include <QGraphicsView>

class RobotGUI : public QWidget
{
    Q_OBJECT

public:
    RobotGUI();

private:
    QGraphicsScene *        myScene;
    QGraphicsView *         myView;
    QList<WSObject> *       currentList;
    RobotLimits             limits;

    float                   xScale;
    float                   yScale;

    void updateWSObjects(QList<WSObject> * newList);
    void redrawItems();

    QList<WSObject> generateWSObject(int n);

};

#endif // ROBOTGUI_H
