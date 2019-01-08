#ifndef OBSLINEIMAGE_H
#define OBSLINEIMAGE_H

#include <QGraphicsItem>
#include <QtMath>
#include <QPoint>

class ObservedLine : public QGraphicsItem
{
public:
    ObservedLine();

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

    QPointF  start;
    QPointF  end;
};

#endif // OBSLINEIMAGE_H
