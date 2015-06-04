#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include "../Linemod/OnlineGrabber.h"
#include "../Linemod/common.h"

//TODO
// Reussir à bien detecter un des templates .pcd
// Dans generation de pcd, on fait rows puis cols, genere peut etre pcd de travers.
//   voir si pas besoin d'inverser.
// Pb: mask width is 480 oO, should be 640.

using namespace std;
using namespace pcl;

LinemodWrap lmw;
vector<double> detection_score;

// rosrun linemod_ros detect_test_offline /home/gmanfred/devel/datasets/my_objects/linemod/milk/small/views10/00025_cloud.pcd models.txt
int main(int argc, char** argv) {
    assert(argc == 3 && "Usage: train_test cloud_in model_in");

    PointCloudType::Ptr cloud_in (new PointCloudType);
    if (io::loadPCDFile<PointType> (argv[1], *cloud_in) == -1) {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return -1;
    }
    cout << "Loaded " << argv[1] << "." << endl;
    cout << "Size: " << cloud_in->width << "/" << cloud_in->height << endl;
    
    int num_objects = lmw.load(argv[2]);
    cout << "Loaded " << num_objects << " objects." << endl;

    vector<PointXYZ> min, max;
    detection_score = lmw.detect(cloud_in, min, max);
    
    for (size_t i = 0; i < detection_score.size(); ++i)
        cout << detection_score[i] << endl;

	return 0;
}
