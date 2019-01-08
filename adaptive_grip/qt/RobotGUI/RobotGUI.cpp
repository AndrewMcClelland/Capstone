#include "RobotGUI.h"

#include <QDebug>
#include <math.h>

#define SCREEN_RATIO 5.0
#define SCENE_WIDTH 600
#define SCENE_HEIGHT 360

#define BUFFER_AMOUNT 135

RobotGUI::RobotGUI() :
    currentList(NULL)
{
    // Initialize both the view and the scene
    QGraphicsScene * scene = new QGraphicsScene;
    QGraphicsView * view = new QGraphicsView;

    // Set the scene, and the scene's size
    myScene = scene;
    myScene->setSceneRect(-BUFFER_AMOUNT/2, -BUFFER_AMOUNT/2, SCENE_WIDTH + BUFFER_AMOUNT, SCENE_HEIGHT+BUFFER_AMOUNT);

    // TEST FUNCTION
    QList<WSObject> someList = generateWSObject(10);
    currentList = &someList;
    updateWSObjects(&someList);
    redrawItems();
    updateWSObjects(currentList);

    // Set the view, and resize the view to the dimensions of its parent scene
    myView = view;
    myView->setScene(myScene);
    myView->resize(myScene->width() + BUFFER_AMOUNT, myScene->height() + BUFFER_AMOUNT);
    myView->setWindowTitle("Gantry Robot GUI");

    myView->show();
}

/**
 * Updates the list of WSObjects that is currently stored within the RobotGUI
 * @param newList is an updated list of WSObjects that is passed by a user
 */
void RobotGUI::updateWSObjects(QList<WSObject> * newList)
{
    QList<WSObject> nList = *newList;
    QList<WSObject> cList = *currentList;

    // The list has been updated. Their sizes are different
    if (newList->size() != cList.size()) {
        currentList = newList;
    }

    // If any of the WSObjects have changed, signal that the lists have updated
    for (int i = 0; i < newList->size(); i++) {
        bool areEqual = (nList[i] == cList[i]);
        if (!areEqual) {
            currentList = newList;
        }
    }
}

/**
 * This should only be called after having called updateWSObjects
 * This function will redraw all items into the scene based on the information
 * given by the WSObjects in the list
 */
void RobotGUI::redrawItems()
{
    QList<WSObject> cList = *currentList;
    QGraphicsRectItem * rect = new QGraphicsRectItem;


    myScene->clear();

    for (int i = 0; i < currentList->size(); i++) {
        ItemsImage * item = new ItemsImage(cList[i]);
        item->setPos(cList[i].x_position / SCREEN_RATIO, cList[i].y_position / -SCREEN_RATIO);
        myScene->addItem(item);
    }

    rect->setRect(0, 0, SCENE_WIDTH, SCENE_HEIGHT);
    myScene->addItem(rect);
}

/**
 * Function to help test the main functionality of the program
 * @param n
 * @return
 */
QList<WSObject> RobotGUI::generateWSObject(int n)
{
    QList<WSObject> list;

    for(int i = 0; i < n; i++) {
        WSObject wsobject(qrand() % 3000, qrand() % 1795*(-1), 100);
        wsobject.r_display = qrand() % 256;
        wsobject.b_display = qrand() % 256;
        wsobject.g_display = qrand() % 256;
        wsobject.x_obs_position = qrand() % 3000;
        wsobject.y_obs_position = qrand() % 1795*(-1);
        list.append(wsobject);
    }

    return list;
}






