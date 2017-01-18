#include "visuallocalization.h"

VisualLocalization::VisualLocalization(ros::NodeHandle _nh) :
    m_nh(_nh)
{
    m_detectedTpl.clear();
    m_detectedTpl.resize(2);
    m_pixelHom.setZero();
    m_worldHom.resize(4,1);
    m_K <<  1.1068, 0.0005, 0.7045,
             0, 1.1123, 0.3601,
             0, 0, 0.0010;
    m_K = m_K * 1000;
    m_subTplDetection = m_nh.subscribe("tpl_detect", 1, &VisualLocalization::cbTplDetect, this);
    m_subMap = m_nh.subscribe("map",1,&VisualLocalization::cbMap, this);
}
void VisualLocalization::cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int minIndex = ceil((min_scan_angle - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((max_scan_angle - scan->angle_min) / scan->angle_increment);
    for (auto it: m_detectedTpl)
    {
        if(scan->ranges[it.theta]!=INFINITY && it.theta <= maxIndex && it.theta >= minIndex){
            it.distance = scan->ranges[it.theta];
        }
    }
    ros::spinOnce();
}

void VisualLocalization::cbMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    int width = msg->info.width;
    int height = msg->info.height;
    if ((width < 3) || (height < 3) ){
          ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", width, height);
          return;
    }
    ros::spinOnce();
}

void VisualLocalization::cbTplDetect(const geometry_msgs::PoseArray::ConstPtr &msg) // MSG TYPE TO BE DEFINED BY BENNI
{
    // pxl points from Benni
    Vector3f pxlHom1, pxlHom2 ;
    double theta1=0, theta2=0;
    pxlHom1(1) = 1;//m_tplInPixels.at(0).x = msg[0].poses.position.x;
    pxlHom1(2) = 1;//m_tplInPixels.at(0).y = msg[0].poses.position.y;
    pxlHom1(3) = 1;
    pxlHom2(1) = 1;//m_tplInPixels.at(1).x = msg[1].poses.position.x;
    pxlHom2(2) = 1;//m_tplInPixels.at(1).y = msg[1].poses.position.y;
    pxlHom2(3) = 1;
    projectPixelToCamera(pxlHom1);
    m_detectedTpl.at(0).theta = convertPointToAngle(m_detectedTpl.at(0)); //
    m_detectedTpl.at(1).theta = convertPointToAngle(m_detectedTpl.at(1)); //
    localizeCar();
    ros::spinOnce();
}

void VisualLocalization::localizeCar()
{
    m_carPosition.u = m_detectedTpl.at(0).distance * cos( m_detectedTpl.at(0).theta * M_PI/180) + m_detectedTpl.at(0).u;
    double arg = pow(m_detectedTpl.at(1).distance,2) - pow((m_carPosition.u - m_detectedTpl.at(0).u),2);
    m_carPosition.v = m_detectedTpl.at(1).v - sqrt(arg);
    // convertPoseToMeters
}

void VisualLocalization::broadcastCameraFrame()
{
    tf::TransformBroadcaster broadcaster;
    tf::Transform transf;
    transf.setOrigin(tf::Vector3(m_camPositionInBaseLink));
    tf::Quaternion q;
    q.setRPY(0,0,0); //  MEASUREMENTS
    transf.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transf, ros::Time::now(),
                                                   "/base_link","/camera_frame"));

}

void VisualLocalization::projectPixelToCamera(Vector3f _pxlHom)
{
    m_worldHom = m_K.inverse() * _pxlHom;
}

void VisualLocalization::transformQuaternionToR()
{
}

double VisualLocalization::convertPointToAngle(TemplateData & _pt)
{
    return _pt.u * visionSettings::pixelToCamResolution * visionSettings::camToLaserResolution;
}
