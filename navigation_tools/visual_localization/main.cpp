#include <iostream>
#include <ros/ros.h>
#include "src/visuallocalization.h"
using namespace cv;
int main(int argc, char** argv){
    ros::init(argc, argv, "visual_localization");
    ros::NodeHandle nh("~");
    vector<string> lm_names(1);
    lm_names.resize(3);

    lm_names.at(0) = "/home/alex/TAS/beta_test/src/navigation_tools/resources/lm_red.png";
    lm_names.at(1) = "/home/alex/TAS/beta_test/src/navigation_tools/resources/lm_green.png";
    lm_names.at(2) = "/home/alex/TAS/beta_test/src/navigation_tools/resources/lm_blue.png";
   //lm_names.at(1) = "/home/alex/TAS/beta_test/src/navigation_tools/resources/lm_red.png";
    /*nh.getParam("lm_in_map_blue",lm_names.at(1));
    nh.getParam("lm_in_map_green",lm_names.at(2));*/



    // get params
    string tpl_path;
    vector<LoadedTemplateData> loadedTemplates(3); // LOAD TPLS FROM PARAM
    //nh.getParam("tpl_in_map_red",tpl_path);
    tpl_path = "/home/alex/TAS/beta_test/src/navigation_tools/resources/red_star.png";
    loadedTemplates.at(0).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(0).startype = StarType::RED;
    tpl_path = "/home/alex/TAS/beta_test/src/navigation_tools/resources/green_star.png";
    loadedTemplates.at(1).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(1).startype = StarType::GREEN;
    tpl_path = "/home/alex/TAS/beta_test/src/navigation_tools/resources/blue_star.png";
    loadedTemplates.at(2).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
    loadedTemplates.at(2).startype = StarType::BLUE;
    //nh.getParam("tpl_in_map_blue",tpl_path);
   // tpl_path = "/home/alex/TAS/beta_test/src/navigation_tools/resources/blue_star.png";
   // loadedTemplates.at(1).src = imread(tpl_path,CV_LOAD_IMAGE_GRAYSCALE);
//    loadedTemplates.at(1).startype = StarType::BLUE;


    VisualLocalization vl(nh, lm_names,loadedTemplates);
    vl.localize();


    return 0;
}

