#include "itemdisplaywindow.h"
#include <iostream>
ItemDisplayWindow::ItemDisplayWindow(QWidget *parent) : QWidget(parent)
{
      setAutoFillBackground(true);
      select_index = -1;
}

int
ItemDisplayWindow::get_index()
{
   return select_index;
}

void
ItemDisplayWindow::cycle()
{
    bool forward = true;
   if (!m_objects) {
      select_index = -1;
   } else if (m_objects->size() == 0) {
      select_index = -1;
   } else if (select_index == -1) {
      select_index = 0;
   } else {
      select_index = (forward) ? select_index + 1 : select_index - 1;
      select_index %= m_objects->size();
   }

   using std::cout;
   using std::endl;
   cout << "select_index = " << select_index << endl;
   update();

}

void
ItemDisplayWindow::update_robot_position(const RobotPosition & pos)
{
   robot_position = pos;
}

void 
ItemDisplayWindow::resizeEvent(QResizeEvent * event)
{
   // define scale_factor here.
   x_scale_factor = (limits.max().x - limits.min().x) / ((float) size().width());
   y_scale_factor = (limits.max().y - limits.min().y) / ((float) size().height());
}

void
ItemDisplayWindow::paintEvent(QPaintEvent * event)
{
   QPainter painter(this);
   QRectF background(0.0, 0.0, size().width(), size().height());
   painter.fillRect(background, Qt::white);

   const float radius = 10.0;
   float gantry_x_draw, gantry_y_draw;

   convert_to_xy(robot_position, gantry_x_draw, gantry_y_draw);
   painter.setPen(QPen(Qt::black));
   painter.drawEllipse(gantry_x_draw - radius, gantry_y_draw - radius, radius*2, radius*2);

   // If the m_objects array hasn't been allocated, return.
   if (!m_objects) {
      select_index = -1;
      return;
   } else if (m_objects->size() == 0) {
      select_index = -1;
      return;
   }

   QPen     defaultPen(Qt::red);
   painter.setPen(defaultPen);


   typedef std::vector<WSObject>::const_iterator Iterator;



   float min_object_x = limits.min().x;
   float min_object_y = limits.min().y;
   
   for (Iterator i = m_objects->begin(); i != m_objects->end(); i++)
   {

      // Change the pen color using the WSObject's r_display, g_display, b_display..
      painter.setPen(QPen(QColor(i->r_display, i->g_display, i->b_display)));

      float draw_at_x = (((*i).x_position - min_object_x) / x_scale_factor) - radius;
      float draw_at_y = (((*i).y_position - min_object_y) / y_scale_factor) - radius;
      painter.drawEllipse(draw_at_x, draw_at_y, radius*2, radius*2);
   }

   if (select_index >= 0)
   {
      float radius_big = radius * 1.15;
      painter.setPen(QPen(Qt::red));
      float draw_at_x = (((*m_objects)[select_index].x_position - min_object_x) / x_scale_factor) - radius_big;
      float draw_at_y = (((*m_objects)[select_index].y_position - min_object_y) / y_scale_factor) - radius_big;
      painter.drawEllipse(draw_at_x, draw_at_y, radius_big*2, radius_big*2);
   }


}


void 
ItemDisplayWindow::convert_to_xy(const RobotPosition & rp, float & x, float & y)
{
   x = (rp.x - limits.min().x) / x_scale_factor;
   y = (rp.y - limits.min().y) / y_scale_factor;
}
