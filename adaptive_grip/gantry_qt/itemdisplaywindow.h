#ifndef ITEMDISPLAYWINDOW_H
#define ITEMDISPLAYWINDOW_H

#include <QtWidgets/QWidget>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtDesigner/QtDesigner>
#include "WSObject.h"

#include "RobotLimits.h"



class QDESIGNER_WIDGET_EXPORT ItemDisplayWindow : public QWidget
{
    Q_OBJECT
public:
    explicit ItemDisplayWindow(QWidget *parent = 0);

    void                      init_display();
    std::vector<WSObject> *   m_objects;

    int                       get_index();
    void                      update_robot_position(const RobotPosition & pos);

protected:

    void     paintEvent(QPaintEvent *);
    void     resizeEvent(QResizeEvent *);


private:

    float                     x_scale_factor;
    float                     y_scale_factor;
    int                       select_index;
    int                       win_width;
    int                       win_height;
    RobotLimits               limits;
    RobotPosition             robot_position;

    void convert_to_xy        (const RobotPosition & rp, float & x, float & y);

signals:

public slots:

   void     cycle();
};

#endif // ITEMDISPLAYWINDOW_H
