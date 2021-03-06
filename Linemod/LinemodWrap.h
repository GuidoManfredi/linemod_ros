#pragma once

#include <pcl/io/pcd_io.h>
//#include <pcl/recognition/linemod.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"

//TODO 
//      Voir pourquoi models avec plus de 750 templates ne sont pas chargés.
//      Voir pourquoi nombre de detections change de 3 à 4
//      Voir dans bestDetections comment rendre des vecteurs de taille variable.
//      Verifier que la creation de object_cloud est bien faite, dans traintemplate (remplissage du header).
//      Remplacer l'appel system de TAR par une librairie adpatée (semble compliqué).
class LinemodWrap {
  public:
    LinemodWrap ();
    void setGradientMagThresh (float grad_mag_thresh);
    void setDetectionThresh (float detect_thresh);

    bool loadCloud (const std::string &filename, PointCloudType &cloud);

    /// DETECTION
    std::vector<double> detect (const PointCloudType::ConstPtr cloud, std::vector<pcl::PointXYZ> &min, std::vector<pcl::PointXYZ> &max);

    /// TRAINING
    // train from real views
    void trainTemplateOnline (const PointCloudType::ConstPtr &input, const std::vector<bool> &foreground_mask);
    // train from virtual object (already masked)
    void trainTemplateVirtual (const PointCloudType::ConstPtr &input, cv::Mat foreground_mask);

    void saveLastTemplate(PointCloudType template_cloud, std::string pcd, std::string sqmmt);
    void saveTemplates (std::vector<std::string> filepaths);
    void save (std::string filepath);
    // There should be no empty line at the end of the file.
    int load (std::string list_path);
    bool loadModel (std::string lmt_filename, int object_id);
    
    void setNumObjects(int num_objects);
//  private:
    int num_objects_;
};
