#ifndef ITEMSIMAGE_H
#define ITEMSIMAGE_H

#include "../../headers/WSObject.h"
#include "ObservedLine.h"

#include <QGraphicsObject>
#include <QDebug>
#include <QPoint>

class ItemsImage : public QGraphicsObject
{
    Q_OBJECT

public:
    ItemsImage(WSObject wso);

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

    QPointF         observed;
    QPointF         position;
    ObservedLine *  myLine;

public slots:
    void mousePressEvent(QGraphicsSceneMouseEvent * event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

private:
    QColor      color;
    int         id;

};

#endif // ITEMSIMAGE_H
