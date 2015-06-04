#pragma once

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "LinemodWrap.h"
#include "common.h"
// TODO
//  Change name to OnlineModeller.
//  Enlever points juste au dessus du plan (pour virer turntable).
//  Acceder a lmw_.num_objects_ de maniere plus propre.
class OnlineGrabber {
  public:
    OnlineGrabber(float grad_mag_tresh, float detect_thresh);
    // Find the object under modelling
    std::vector<double> detect(const PointCloudType::ConstPtr &input, std::vector<pcl::PointXYZ> &min, std::vector<pcl::PointXYZ> &max);
    // Computes the foreground mask, add the template to the underlying linemod and
    //  save the templates in sqmmt and pcd files.
    void grab (const PointCloudType::ConstPtr &input);
    // Remove points too close, too far, segment the main plane in view and remove
    //  point too high above this plane. Returns a mask.
    std::vector<bool> maskForegroundPoints (const PointCloudType::ConstPtr & input,
                                             float min_depth, float max_depth,
                                             float min_height, float max_height);
    void save (std::string filepath);

  private:
    int num_templates_;
    LinemodWrap lmw_;
    float min_depth_, max_depth_, min_height_, max_height_;
};
