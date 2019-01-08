#include "chartview.h"


ChartView::ChartView(QWidget *parent) :
    QChartView(new QChart(), parent)
{
    /*
    QScatterSeries *series0 = new QScatterSeries();

    series0->setName("test1");
    series0->append(0, 3);
    series0->append(3, 0);
    series0->append(3, 3);

    chart()->addSeries(series0);
    chart()->setTitle("Test Title");
    chart()->createDefaultAxes();
    chart()->setDropShadowEnabled(false);
    */
}

void
ChartView::updateSeries()
{
    // read m_series

	 // TODO clear the chart view

#ifndef TEST_RECT
	 typedef std::vector<Coordinate>::const_iterator	CoordIt;

    QScatterSeries * points_2d = new QScatterSeries();
    for (CoordIt it = surface->coordinates.begin(); it != surface->coordinates.end(); ++it)
	 {
		// Fill a series

        points_2d->append(it->x, it->y);
	 }
     points_2d->setMarkerSize(5.0);
     points_2d->setColor(QColor(255,0,0));

	 QScatterSeries * convexHull = new QScatterSeries();

	 // Assume the surface is already initialized.
////surface->convex_hull();
////surface->calculateMinimumAreaRect();
	
	 // Plot the Convex Hull Dataset
	 for (CoordIt it = surface->hull.begin(); it != surface->hull.end(); ++it)
	 {
		convexHull->append(it->x, it->y);
	 }
     convexHull->setMarkerSize(15.0);
     convexHull->setColor(QColor(0,0,255));

     std::cout << "Size of convex hull: " << surface->hull.size() << std::endl;

	 // Plot the minimum-area bounding rectangle
	 QScatterSeries * boundingRect = new QScatterSeries();
	 
	 std::vector<Coordinate> boundingVect = surface->minimumAreaRectangle.corners();

	 for (CoordIt it = boundingVect.begin(); it != boundingVect.end(); ++it)
	 {
		 boundingRect->append(it->x, it->y);
		 boundingRect->setMarkerSize(20.0);
		 boundingRect->setColor(QColor(0, 255, 0));
	 }
	 boundingRect->append(surface->minimumAreaRectangle.center.x, surface->minimumAreaRectangle.center.y);
	 chart()->addSeries(boundingRect);
	 
	 chart()->addSeries(convexHull);
	 chart()->addSeries(points_2d);
	 chart()->setTitle("2D Points");
	 chart()->createDefaultAxes();
	 chart()->setDropShadowEnabled(false);
#else
	 
	 // Declare a surface...
	 typedef std::vector<Coordinate>::const_iterator	CoordIt;
	 SurfaceMap testSm;

	 // Populate it: 10 points between -5 and 5, just to get things going
     for (int i = 0; i < 7; i++)
	 {
		int rand_x = qrand() % 10 - 5;
		int rand_y = qrand() % 10 - 5;

		// add a coordinate at this location
		testSm.coordinates.push_back(Coordinate(rand_x, rand_y));
	 }

	 // Plot its points

    QScatterSeries * points_2d = new QScatterSeries();
    for (CoordIt it = testSm.coordinates.begin(); it != testSm.coordinates.end(); ++it)
	 {
		// Fill a series
		points_2d->append(it->x, it->y);
	 }
     points_2d->setMarkerSize(5.0);
     points_2d->setColor(QColor(255,0,0));

	 QScatterSeries * convexHull = new QScatterSeries();
	 testSm.convex_hull();

	// lazy way to get a nicer view
	 points_2d->append(-8.0, -8.0);
	 points_2d->append( 8.0,  8.0);
	
	 // Plot the Convex Hull Dataset
	 for (CoordIt it = testSm.hull.begin(); it != testSm.hull.end(); ++it)
	 {
		convexHull->append(it->x, it->y);
	 }
     convexHull->setMarkerSize(15.0);
     convexHull->setColor(QColor(0,0,255));

     std::cout << "Size of convex hull: " << testSm.hull.size() << std::endl;
	 // Populate the "convex hull" dataset

	 // Plot the minimum-area bounding rectangle
	 QScatterSeries * boundingRect = new QScatterSeries();
	 
	 testSm.calculateMinimumAreaRect();
	 std::vector<Coordinate> boundingVect = testSm.minimumAreaRectangle.corners();

	 for (CoordIt it = boundingVect.begin(); it != boundingVect.end(); ++it)
	 {
		 boundingRect->append(it->x, it->y);
		 boundingRect->setMarkerSize(20.0);
		 boundingRect->setColor(QColor(0, 255, 0));
	 }
	 boundingRect->append(testSm.minimumAreaRectangle.center.x, testSm.minimumAreaRectangle.center.y);


	 chart()->addSeries(boundingRect);
	 chart()->addSeries(convexHull);
	 chart()->addSeries(points_2d);
	 chart()->setTitle("2D Points");
	 chart()->createDefaultAxes();
	 chart()->setDropShadowEnabled(false);

	 // Plot its convex hull

#endif
}


