#ifndef CHARTVIEW_H
#define CHARTVIEW_H

#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLegendMarker>
#include <QtGui/QPainter>
#include "SurfaceMap.h"



QT_CHARTS_USE_NAMESPACE

class ChartView : public QChartView
{
    Q_OBJECT
public:
    explicit ChartView(QWidget *parent = 0);
	 
	 SurfaceMap * surface;
    
signals:
    
public slots:
    
    void updateSeries();
};

#endif // CHARTVIEW_H
