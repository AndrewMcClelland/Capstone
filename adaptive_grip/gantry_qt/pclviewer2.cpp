#include "pclviewer2.h"
#include "ui_pclviewer2.h"
#include "CameraFile.h"

#include <sstream>

PCLViewer2::PCLViewer2(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer2),
    mutex(new Mutex()),
    cloud(new PointCloud),
    camera(new CameraFile<Point>(cloud, mutex))
//  logger(new QLogger(ui->textDisplay)),
//  _engine(cloud, mutex, logger)
{
    ui->setupUi(this);
    this->setWindowTitle("PCL Viewer");

    // TODO Load the cloud
    logger = new QLogger(ui->textDisplay);
    _engine = new Engine(cloud, mutex, logger);

    // Note to self: need to create without an interactor, else "setupInteractor()" call will fail.
    // That's the significance of the added "false" in the viewer's constructor
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

//  camera->retrieve();

    viewer->addPointCloud(_engine->m_vis_cloud, "VisualizationCloud");

    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    // Useful: PCL Viewer has a function PCLViewer::resetCamera();

    
    // CONNECT SLOTS
    CHECKED_CONNECT(ui->captureImageButton,  SIGNAL(pressed()), this, SLOT(capture_image()))

    CHECKED_CONNECT(ui->getFloorButton,      SIGNAL(pressed()), this, SLOT(detect_floor()))
    CHECKED_CONNECT(ui->findClusterButton,   SIGNAL(pressed()), this, SLOT(find_clusters()))
    CHECKED_CONNECT(ui->getDistanceButton,   SIGNAL(pressed()), this, SLOT(move_claw()))

    CHECKED_CONNECT(ui->nextItemButton,   SIGNAL(pressed()), ui->itemDisplayWindow, SLOT(cycle(true)))
    CHECKED_CONNECT(ui->prevItemButton,   SIGNAL(pressed()), ui->itemDisplayWindow, SLOT(cycle(false)))

//  CHECKED_CONNECT(ui->floorAlphaSlider,    SIGNAL(
}

PCLViewer2::~PCLViewer2()
{
    delete ui;
}


void /*slot*/
PCLViewer2::capture_image()
{
   logger->log("[Capture Image] button pressed.");
   _engine->get_image();
   _update_cloud();
}

void /*slot*/
PCLViewer2::detect_floor()
{
   logger->log("[Detect Floor Plane] button pressed.");
   _engine->get_floor();
   _update_cloud();
}

void /*slot*/
PCLViewer2::find_clusters()
{
   logger->log("[Find Clusters] button pressed.");
   _engine->get_clusters();

   logger->log("PCLViewer2::find_clusters() -- Adding CentroidsCloud..");
   viewer->addPointCloud(_engine->m_vis_centroids, "CentroidsCloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "CentroidsCloud");
   viewer->updatePointCloud(_engine->m_vis_centroids, "CentroidsCloud");

   // Add the Claw's Cluster : red line
   const Eigen::Vector4f tmp_plane = _engine->m_plane_filter->get_plane_coefficients();
   viewer->addLine(_engine->m_claw_cluster.m_location, _engine->m_claw_cluster.get_plane_projection(tmp_plane), "ClawClusterLine");

   // Rest of the clusters : blue line
   int i =0 ;
   for (std::vector<Cluster<Engine::Point> >::iterator it = _engine->m_clusters.begin();
         it != _engine->m_clusters.end(); ++it)
   {
      std::stringstream s;
      Cluster<Engine::Point> c(*it);
      s << "Line" << i;

      std::stringstream msg;
      msg << "Adding Line: <" << s.str() + ">";
      logger->log(msg);

      viewer->addLine(c.m_location, c.get_plane_projection(tmp_plane), s.str());
      ++i;
   }

   _update_cloud();
}

void /*slot*/
PCLViewer2::move_claw()
{
   logger->log("[Move Claw] button pressed.");
   _engine->get_movement();
   _update_cloud();
}

void
PCLViewer2::_update_cloud()
{
   logger->log("Updating cloud.");
   viewer->updatePointCloud(_engine->m_vis_cloud, "VisualizationCloud");
}
