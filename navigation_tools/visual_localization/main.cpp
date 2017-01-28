#include <iostream>
#include <ros/ros.h>
#include "src/visuallocalization.h"
using namespace cv;
int main(int argc, char** argv){
    ros::init(argc, argv, "visual_localization");
    ros::NodeHandle nh("~");
    vector<string> lm_names(1);
    nh.getParam("lm_in_map_red",lm_names.at(0));
    /*nh.getParam("lm_in_map_blue",lm_names.at(1));
    nh.getParam("lm_in_map_green",lm_names.at(2));*/



    // get params
    string tpl_path;
    vector<LoadedTemplateData> loadedTemplates(1); // LOAD TPLS FROM PARAM
    nh.getParam("tpl_in_map_red",tpl_path);
    loadedTemplates.at(0).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(0).startype = StarType::RED;
    VisualLocalization vl(nh, lm_names,loadedTemplates);
    vl.localize();


    return 0;
}

