#include <iostream>
#include <ros/ros.h>
#include "src/visuallocalization.h"
using namespace cv;

/**
 * @brief main Load landmark patches and templates from the parameters defined in the
 * launch file (localization.launch) and start the visual localization process.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "visual_localization");
    ros::NodeHandle nh("~");
    vector<string> lm_names(3);

    nh.getParam("lm_in_map_red",lm_names.at(0));
    nh.getParam("lm_in_map_green",lm_names.at(1));
    nh.getParam("lm_in_map_blue",lm_names.at(2));

    // get params
    string tpl_path;
    vector<LoadedTemplateData> loadedTemplates(3);
    nh.getParam("tpl_in_map_red",tpl_path);
    loadedTemplates.at(0).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(0).startype = StarType::RED;
    nh.getParam("tpl_in_map_green",tpl_path);
    loadedTemplates.at(1).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(1).startype = StarType::GREEN;
    nh.getParam("tpl_in_map_blue",tpl_path);
    loadedTemplates.at(2).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(2).startype = StarType::BLUE;

    bool use_map_img=false;
    string map_path;
    if (nh.getParam("use_map_image",use_map_img))
    {
        ROS_INFO("Using JPG file of map.");
    }
    nh.getParam("map_image",map_path);


    VisualLocalization vl(nh, lm_names,loadedTemplates,use_map_img,map_path);
    vl.localize();


    return 0;
}

