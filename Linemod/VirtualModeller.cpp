#include "VirtualModeller.h"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

VirtualModeller::VirtualModeller(float grad_mag_thresh, float detect_thresh) {
    lmw_.setGradientMagThresh(grad_mag_thresh);
    lmw_.setDetectionThresh (detect_thresh);
}

int VirtualModeller::model(string viewspath, string outpath) {
    vector<string> files = loadFolder(viewspath);
    //cout << files.size() << endl;
    for (size_t i = 0; i < files.size(); ++i) {
        PointCloudType::Ptr pcd(new PointCloudType());
        Mat* mask = new Mat();
        loadFile(files[i], pcd, mask);
        //cout << "Adding view (" << pcd->width << "," << pcd->height << ")" << endl;
        addView(pcd, *mask);
    }
    save(files, outpath);
    return files.size();
}

vector<string> VirtualModeller::loadFolder(std::string folderpath) {
    vector<string> filepaths;
    path bf_images_path(folderpath);
    if ( is_directory(bf_images_path) ) {
        typedef vector<path> vec;
        vec v;
        copy(directory_iterator(bf_images_path), directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end());
        // for each file in the folder
        for (vec::const_iterator it (v.begin()); it != v.end(); ++it) {
            if (strcmp(it->extension().string().c_str(),".pcd") == 0) {
                vector<string> parts;
                boost::split(parts,it->stem().string(),boost::is_any_of("_"));
                
                string filepath_no_ext = it->parent_path().string() + "/" + parts[0];
                filepaths.push_back(filepath_no_ext);
            }
        }
    } else {
        cout << "Specified path do not lead to a folder (maybe it's a file?)" << endl;
    }
    return filepaths;
}

void VirtualModeller::loadFile(std::string filepath, PointCloudType::Ptr pcd, cv::Mat* mask_image) {
    string pcd_file = filepath + "_cloud.pcd";
    cout << "Loading " << pcd_file << endl;
    if(pcl::io::loadPCDFile<PointType> (pcd_file, *pcd) == -1){ // load the file
        cout << "Error loading " << pcd_file << endl;
        return;
    }
    
    string mask_file = filepath + "_mask.png";
    *mask_image = imread(mask_file, CV_LOAD_IMAGE_GRAYSCALE);
}

void VirtualModeller::addView(const PointCloudType::ConstPtr &input, Mat mask) {
    lmw_.trainTemplateVirtual(input, mask);
}

/*
void VirtualModeller::loadFile(std::string filepath, PointCloudType::Ptr pcd, std::vector<bool> &mask) {
    string pcd_file = filepath + "_cloud.pcd";
    cout << "Loading " << pcd_file << endl;
    if(pcl::io::loadPCDFile<PointType> (pcd_file, *pcd) == -1){ // load the file
        cout << "Error loading " << pcd_file << endl;
        return;
    }
    
    string mask_file = filepath + "_mask.png";
    Mat mask_image = imread(mask_file, CV_LOAD_IMAGE_GRAYSCALE);
    
    mask.clear();
    for (size_t i = 0; i < mask_image.rows; ++i) {
        for (size_t j = 0; j < mask_image.cols; ++j) {
            if (mask_image.at<uchar>(i,j) == 255)
                mask.push_back(true);
            else
                mask.push_back(false);
        }
    }
    int n = 0;
    for (size_t i = 0; n < 1000; ++i) {
        if (mask[i]) {
            cout << i << " ";
            ++n;
        }
    }
    cout << endl;
    
    pcd->width = mask_image.rows;
    pcd->height = mask_image.cols;   
}
*/

// Add a view to the model under construction
/*
void VirtualModeller::addView(const PointCloudType::ConstPtr &input, const std::vector<bool> &mask) {
    lmw_.trainTemplate1(input, mask);
}
*/

// Save the constructed model
void VirtualModeller::save (vector<string> viewspath, std::string output) {
    lmw_.saveTemplates(viewspath);
    lmw_.save(output);
}

