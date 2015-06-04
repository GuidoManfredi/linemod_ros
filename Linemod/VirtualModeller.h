#pragma once

#include <iostream>

#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include "LinemodWrap.h"
#include "common.h"


class VirtualModeller {
  public:
    VirtualModeller(float grad_mag_tresh, float detect_thresh);
    // Opens a folder, load a view and add it to the model under construction
    int model(std::string folderpath, std::string outpath);
    // Read a directory and returns the names of the files which exist in pcd format.
    std::vector<std::string> loadFolder(std::string folderpath);
    //void loadFile(std::string filepath, PointCloudType::Ptr pcd, std::vector<bool> &mask);
    void loadFile(std::string filepath, PointCloudType::Ptr pcd, cv::Mat *mask_image);
    // Add a view to the model under construction
//    void addView(const PointCloudType::ConstPtr &input, const std::vector<bool> &mask);
    void addView(const PointCloudType::ConstPtr &input, cv::Mat mask);
    // Save the constructed model
    void save (std::vector<std::string> viewspath, std::string output);

  private:
    int num_templates_;
    LinemodWrap lmw_;
};
