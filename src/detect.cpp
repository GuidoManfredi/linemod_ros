#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../Linemod/OnlineGrabber.h"

// TODO
//  Voir si nombre de detections augmente avec le temps. 
//  Si oui, alors besoin de nettoyer un vecteur a chaque iteration.

using namespace std;
using namespace pcl;

LinemodWrap lmw;

visualization::PCLVisualizer viewer("Detection");
vector<double> detection_score;

float grad_mag_threshold = 10.0;
float detection_threshold = 0.8;

void keyboard_cb(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    if (event.getKeySym () == "q" && event.keyDown ()) {
        std::cout << "Exiting." << std::endl;
        viewer.close();
    }

    if (event.getKeySym () == "s" && event.keyDown ()) {
        grad_mag_threshold += 1.0;
        lmw.setGradientMagThresh(grad_mag_threshold);
        cout << "Gradient magnitude threshold: " << grad_mag_threshold << endl;
    }
    if (event.getKeySym () == "d" && event.keyDown ()) {
        grad_mag_threshold -= 1.0;
        lmw.setGradientMagThresh(grad_mag_threshold);
        cout << "Gradient magnitude threshold: " << grad_mag_threshold << endl;
    }

    if (event.getKeySym () == "x" && event.keyDown ()) {
        detection_threshold += 0.05;
        lmw.setDetectionThresh(detection_threshold);
        cout << "Detection threshold: " << detection_threshold << endl;
    }
    if (event.getKeySym () == "c" && event.keyDown ()) {
        detection_threshold -= 0.05;
        lmw.setDetectionThresh(detection_threshold);
        cout << "Detection threshold: " << detection_threshold << endl;
    }
}

void cloud_cb(const PointCloudType::ConstPtr& msg) {
	if (!viewer.updatePointCloud (msg, "single_cloud"))
		viewer.addPointCloud (msg, "single_cloud");

    vector<PointXYZ> min, max;
    detection_score = lmw.detect(msg, min, max);

    // Show result
    for (size_t n = 0; n < detection_score.size(); ++n ) {
        //cout << "Object " << n << " : " << detection_score[n] << endl;
        stringstream id;
        id << n;
        viewer.removeShape (id.str(), 0);
        //cout << min[n] << endl;
        //cout << max[n] << endl;

        viewer.addCube (min[n].x, max[n].x, min[n].y, max[n].y, min[n].z, max[n].z,
                        1.0, 1.0, 1.0, id.str());
    }
    cout << endl;
}

// rosrun linemod_ros detect_test /camera/depth_registered/points models.txt
int main(int argc, char** argv) {
    assert(argc == 3 && "Usage: train_test cloud_in model_in");

	ros::init(argc, argv, "linemod_online_train");
	ros::NodeHandle n;
	ros::Subscriber sub_clouds = n.subscribe(argv[1], 1, cloud_cb);

	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();
    viewer.registerKeyboardCallback(&keyboard_cb, (void*) &viewer);

    int num_objects = lmw.load(argv[2]);
    cout << "Loaded " << num_objects << " objects." << endl;
	ros::Rate loop_rate(10);
	while (ros::ok() && !viewer.wasStopped()) {
	    viewer.spinOnce(1);
		ros::spinOnce();
		loop_rate.sleep ();
	}

	return 0;
}


