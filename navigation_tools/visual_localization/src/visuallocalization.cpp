#include "visuallocalization.h"

VisualLocalization::VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names) :
    m_nh(_nh), m_gotMap(false)
{
    m_detectedTpl.clear();
    m_detectedTpl.resize(2);
    m_pixelHom.setZero();
    m_worldHom.resize(4,1);
    m_K <<  1.1068, 0.0005, 0.7045,
             0, 1.1123, 0.3601,
             0, 0, 0.0010;
    m_K = m_K * 1000;

    m_landmarks.resize(2);
    m_landmarks.at(0).src = imread(_lm_names.at(0),1);
    m_landmarks.at(1).src = imread(_lm_names.at(1),1);
    m_subTplDetection = m_nh.subscribe("/tpl_detect", 1, &VisualLocalization::cbTplDetect, this); // FROM BENNI
    m_subMap = m_nh.subscribe("/map_image/full",1,&VisualLocalization::cbMap, this);
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

void VisualLocalization::cbMap(const sensor_msgs::ImageConstPtr& msg) // hector_compressed_map_transport
{
    try {
        cv_bridge::CvImageConstPtr imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (imgPtr == NULL)
        {
            ROS_WARN("Received NULL image.");
        }
        m_mapImg = imgPtr->image;
        locateLandmarksInMap();
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        exit(-1);
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
    m_detectedTpl.at(0).map_u = 2135; //can be found with ORB
    m_detectedTpl.at(0).map_v = 1987;
    m_detectedTpl.at(1).map_u = 2218;
    m_detectedTpl.at(1).map_v = 2010;
    for (auto it: m_detectedTpl)
    {
        it.theta = convertPointToAngle(it); //
    }
    localizeCar();
    ros::spinOnce();
}

void VisualLocalization::locateLandmarksInMap()
{
    if (!m_gotMap)
    {
        m_gotMap = true;
        m_pLandmarkMatcher = new LandmarkMatcher(m_mapImg,m_landmarks);
        cout << "center of tpl1 " << m_landmarks.at(0).center <<endl;
        cout << "center of tpl2 " << m_landmarks.at(1).center <<endl;

    }
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
