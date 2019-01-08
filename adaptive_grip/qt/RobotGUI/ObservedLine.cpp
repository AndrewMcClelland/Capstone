#include "ObservedLine.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

ObservedLine::ObservedLine()
    : start(0,0),
      end(0,0)
{
}

QRectF ObservedLine::boundingRect() const
{
    return QRectF(0, 0, this->scene()->width(), this->scene()->height());
}

void ObservedLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    painter->setPen(Qt::black);

    painter->drawLine(start, end);
    painter->drawEllipse(end, 10, 10);
    painter->drawText(start.x() + 10, start.y() + 10, QString("Position: " + QString::number(start.x() * 5)) +
                                                ", " + QString::number(start.y()*-5));
    painter->drawText(end.x() + 10, end.y() + 10, QString("Observed From: " + QString::number(end.x()*5)) +
                                                ", " + QString::number(end.y()*-5));
}

