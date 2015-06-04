#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../Linemod/OnlineGrabber.h"

using namespace std;
using namespace pcl;

OnlineGrabber og(10.0, 0.95);
visualization::PCLVisualizer viewer("Training");
float detection_thresh = 0.98;
vector<double> detection_score(1);
bool running = false;

void keyboard_cb(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    if (event.getKeySym () == "q" && event.keyDown ()) {
        std::cout << "Exiting." << std::endl;
        viewer.close();
    }

    if (event.getKeySym () == "s" && event.keyDown ()) {
        if (running) {
            running = false;
            std::cout << "Stopping modelling." << std::endl;
        } else {
            running = true;
            std::cout << "Starting modelling." << std::endl;
        }
    }
}

void cloud_cb(const PointCloudType::ConstPtr& msg) {
	if (!viewer.updatePointCloud (msg, "single_cloud"))
    	viewer.addPointCloud (msg, "single_cloud");
    	
    if (running) {
        vector<PointXYZ> min, max;
        // Si reconnaissance sous seuil
        if (detection_score[0] < detection_thresh) {
            og.grab(msg);
            detection_score = og.detect(msg, min, max);
        } else {
            detection_score = og.detect(msg, min, max);
        }
        cout << "Detection: " << detection_score[0] << endl;

        // Show result
        viewer.removeShape ("cube", 0);
        viewer.addCube (min[0].x, max[0].x, min[0].y, max[0].y, min[0].z, max[0].z);
    }
}

// Se mettre dans un dossier vide, ou seront ecris les templates.
// rosrun linemod_ros train_test /camera/depth_registered/points model.lmt
// rosrun linemod_ros train_test passz/output model.lmt
int main(int argc, char** argv) {
    assert(argc == 3 && "Usage: train_test cloud_in model_out");

	ros::init(argc, argv, "linemod_online_train");
	ros::NodeHandle n;
	ros::Subscriber sub_clouds = n.subscribe(argv[1], 1, cloud_cb);

	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();
    viewer.registerKeyboardCallback(&keyboard_cb, (void*) &viewer);

	ros::Rate loop_rate(10);
	while (ros::ok() && !viewer.wasStopped()) {
	    viewer.spinOnce(1);
		ros::spinOnce();
		loop_rate.sleep ();
	}

    og.save(argv[2]);

	return 0;
}
