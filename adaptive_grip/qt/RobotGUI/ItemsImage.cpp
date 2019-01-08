#include "ItemsImage.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

ItemsImage::ItemsImage(WSObject wso)
    : color (wso.r_display, wso.b_display, wso.g_display),
      id (wso.id)
{
    position.setX(wso.x_position);
    position.setY(wso.y_position);
    observed.setX(wso.x_obs_position);
    observed.setY(wso.y_obs_position);
}

QRectF ItemsImage::boundingRect() const
{
    return QRectF(-15, -15, 30, 30);
}

void ItemsImage::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    painter->setPen(Qt::black);
    painter->setBrush(color);
    painter->drawEllipse(-10, -10, 20, 20);
    painter->drawText(-5, 20, QString::number(id));
    painter->setBrush(Qt::NoBrush);
    painter->setPen(Qt::NoPen);
}

void ItemsImage::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // Initialize and set the
    ObservedLine * line = new ObservedLine();

    QPointF pMapped, oMapped, trueS, trueE;

    // Map their coordinates into our GUI's coordinate system
    pMapped.setX(position.x() / 5.0);
    pMapped.setY(position.y() / -5.0);
    oMapped.setX(observed.x() / 5.0);
    oMapped.setY(observed.y() / -5.0);

    trueS = pMapped;
    trueE = oMapped;

    line->start = trueS;
    line->end = trueE;

    myLine = line;
    line = NULL;
    delete line;

    this->scene()->addItem(myLine);
}

void ItemsImage::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    this->scene()->removeItem(myLine);
    delete this->myLine;
}
