#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "OnlineGrabber.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {
        first_cloud_ = true;
    }

    //void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    void cloud_cb_ (const PointCloudType::ConstPtr &cloud) {
        if (first_cloud_) {
            og.grab(cloud);
            first_cloud_ = false;
        }

        if (!viewer.wasStopped())
            viewer.showCloud (cloud);
    }

    void run () {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        //boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::function<void (const PointCloudType::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);
        interface->start ();

        while (!viewer.wasStopped()) {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    OnlineGrabber og;
    pcl::visualization::CloudViewer viewer;
    bool first_cloud_;
};

int main() {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
}


