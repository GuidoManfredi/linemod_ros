#include "OnlineGrabber.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;

OnlineGrabber::OnlineGrabber(float grad_mag_thresh, float detect_thresh) {
    num_templates_ = 0;
    min_depth_ = 0.3; // meters
    max_depth_ = 2.0;
    max_height_ = 1.0; 
    min_height_ = 0.0;
    lmw_.setGradientMagThresh(grad_mag_thresh);
    lmw_.setDetectionThresh (detect_thresh);
    lmw_.setNumObjects(1);
}


std::vector<double> OnlineGrabber::detect(const PointCloudType::ConstPtr &input, std::vector<pcl::PointXYZ> &min, std::vector<pcl::PointXYZ> &max) {
    return lmw_.detect(input, min, max);
}

void OnlineGrabber::grab (const PointCloudType::ConstPtr & input) {
    cout << "Grabing..." << endl;
    // Segment the foreground object
    std::vector<bool> foreground_mask = maskForegroundPoints (input, 
                                                              min_depth_, max_depth_,
                                                              min_height_, max_height_);
    // Save the masked template cloud (masking with NaNs to preserve its organized structure)
    PointCloudType template_cloud (*input);
    for (size_t i = 0; i < foreground_mask.size (); ++i) {
        if (!foreground_mask[i]) {
            PointType &p = template_cloud.points[i];
            p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
        }
    }
    // Train a template
    lmw_.trainTemplateOnline(input, foreground_mask);
    // Save the template
    stringstream string_num_templates;
    string_num_templates << num_templates_;
    string pcd_savename = "template" + string_num_templates.str() + ".pcd";
    string sqmmt_savename = "template" + string_num_templates.str() + ".sqmmt";
    cout << "Saved to " << pcd_savename << " and " << sqmmt_savename << endl;
    lmw_.saveLastTemplate(template_cloud, pcd_savename, sqmmt_savename);
    ++num_templates_;
}

std::vector<bool> OnlineGrabber::maskForegroundPoints (const PointCloudType::ConstPtr & input,
                                                        float min_depth, float max_depth, 
                                                        float min_height, float max_height) {
    std::vector<bool> foreground_mask (input->size (), false);
    // Mask off points outside the specified near and far depth thresholds
    pcl::IndicesPtr indices (new std::vector<int>);
    for (size_t i = 0; i < input->size (); ++i) {
        const float z = input->points[i].z;
        if (min_depth < z && z < max_depth) {
            foreground_mask[i] = true;
            indices->push_back (static_cast<int> (i));
        }
    }

     // Find the dominant plane between the specified near/far thresholds
    const float distance_threshold = 0.02f;
    const int max_iterations = 500;
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);
    seg.setMaxIterations (max_iterations);
    seg.setInputCloud (input);
    seg.setIndices (indices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    seg.segment (*inliers, *coefficients);

    // Mask off the plane inliers
    for (size_t i = 0; i < inliers->indices.size (); ++i)
        foreground_mask[inliers->indices[i]] = false;

    // Mask off any foreground points that are too high above the detected plane
    const std::vector<float> & c = coefficients->values;
    for (size_t i = 0; i < input->size (); ++i) {
        if (foreground_mask[i]) {
            const pcl::PointXYZRGBA & p = input->points[i];
            float d = fabsf (c[0]*p.x + c[1]*p.y + c[2]*p.z + c[3]);
            //foreground_mask[i] = (d < max_height && d > min_height);
            foreground_mask[i] = (d < max_height);
        }
    }
    return foreground_mask;
}

void OnlineGrabber::save(std::string filepath) {
    lmw_.save(filepath);
}


