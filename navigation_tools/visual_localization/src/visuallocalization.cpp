#include "visuallocalization.h"

VisualLocalization::VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names) :
    m_nh(_nh), m_gotMap(false), m_bLocalizationMode(false)
{
    m_detectedTpl.clear();
    m_detectedTpl.resize(3);
    m_pixelHom.setZero();
    m_worldHom.resize(4,1);
    m_K <<  1.1068, 0.0005, 0.7045,
             0, 1.1123, 0.3601,
             0, 0, 0.0010;
    m_K = m_K * 1000;

    m_landmarks.resize(3);
    m_landmarks.at(0).src = imread(_lm_names.at(0),1);
    m_landmarks.at(1).src = imread(_lm_names.at(1),1);
    m_landmarks.at(2).src = imread(_lm_names.at(2),1);
    m_subTplDetection = m_nh.subscribe("/tpl_detect", 1, &VisualLocalization::cbTplDetect, this); // FROM BENNI
    m_subMap = m_nh.subscribe("/map_image/full",1,&VisualLocalization::cbMap, this);
}

/**
 * @brief VisualLocalization::cbLaserScan get distance measure from Lidar
 * @param scan
 */
void VisualLocalization::cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int minIndex = 0;//ceil((min_scan_angle - scan->angle_min) / scan->angle_increment);
    int maxIndex = 719;//floor((max_scan_angle - scan->angle_min) / scan->angle_increment);
    if (m_bLocalizationMode)
    {
        for (int i=0;i<m_detectedTpl.size();i++)
        {
            unsigned int rangeIndex = getRangeFromAngle(m_detectedTpl.at(i).theta);
            if(scan->ranges[rangeIndex]!=INFINITY &&
                    rangeIndex <= maxIndex &&
                    rangeIndex >= minIndex){
                m_detectedTpl.at(i).distance = scan->ranges[rangeIndex];
            }
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


// *************** BAUSTELLE -> BRAUCHE TEMPLATE MESSAGES******************************
/**
 * @brief VisualLocalization::cbTplDetect callback function that gets invoked when 3 templates are detected
 * @param msg
 */
void VisualLocalization::cbTplDetect(const geometry_msgs::PoseArray::ConstPtr &msg) //TO BE CHANGED
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
    m_detectedTpl.at(0).map_u = 2410; //can be found with tpl matcher
    m_detectedTpl.at(0).map_v = 2013;
    m_detectedTpl.at(1).map_u = 2427;
    m_detectedTpl.at(1).map_v = 1980;
    m_detectedTpl.at(2).map_u = 2448;
    m_detectedTpl.at(2).map_v = 1988;
    for (int i=0; i<m_detectedTpl.size(); i++)
    {
        m_detectedTpl.at(i).theta = convertPointToAngle(m_detectedTpl.at(i)); //
    }
    Point2f currentPose = estimatePosition(m_landmarks, m_detectedTpl.at(0).distance,
                                        m_detectedTpl.at(1).distance,
                                        m_detectedTpl.at(2).distance);

    // if (...)
    m_bLocalizationMode = true; // ERST TRUE SETZEN WENN 3 TEMPLATES ERKANNT WURDEN, ANSONSTEN FALSE
    // else

    ros::spinOnce();
}
// *************** ** ******************************

void VisualLocalization::locateLandmarksInMap()
{
    if (!m_gotMap)
    {
        m_gotMap = true;
        m_pLandmarkMatcher = new LandmarkMatcher(m_mapImg,m_landmarks, m_nh);
        cout << "center of tpl1 " << m_landmarks.at(0).map_coordinates <<endl;
        cout << "center of tpl2 " << m_landmarks.at(1).map_coordinates <<endl;
        cout << "center of tpl3 " << m_landmarks.at(2).map_coordinates <<endl;

    }
}

Point2f VisualLocalization::estimatePosition(vector<LandmarkData> &_lm, float da, float db, float dc)
{
    Point2f estimated_position;

    float x_a = _lm.at(0).map_coordinates.x; // 1. Template
    float y_a = _lm.at(0).map_coordinates.y;

    float x_b = _lm.at(1).map_coordinates.x; // 2. Template
    float y_b = _lm.at(1).map_coordinates.y;

    float x_c = _lm.at(2).map_coordinates.x; // 3. Template
    float y_c = _lm.at(2).map_coordinates.y;

    // ====================== Triangulation solution from non-linear system  ===========================

    estimated_position.y = (x_b - x_c)*(pow(x_b,2) - pow(x_a,2) + pow(y_b,2) - pow(y_a,2) + pow(da,2)-pow(db,2));
    estimated_position.y -= (x_a - x_b)*(pow(x_c,2)-pow(x_b,2) + pow(y_c,2) - pow(y_b,2) + pow(db,2)-pow(dc,2));
    estimated_position.y /= 2*((y_a-y_b)*(x_b-x_c) - (y_b-y_c)*(x_a-x_b));

    estimated_position.x = (y_b - y_c)*(pow(y_b,2)-pow(y_a,2) + pow(x_b,2)-pow(x_a,2) + pow(da,2)-pow(db,2));
    estimated_position.x -= (y_a-y_b)*(pow(y_c,2)-pow(y_b,2) + pow(x_c,2)-pow(x_b,2) + pow(db,2)-pow(dc,2));
    estimated_position.x /= 2*((x_a-x_b)*(y_b-y_c) - (x_b-x_c)*(y_a-y_b));

    return -estimated_position;
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


/**
 * @brief VisualLocalization::convertPointToAngle convert a pixel point to an angle in the FOV
 * @param _pt
 * @return
 */
double VisualLocalization::convertPointToAngle(TemplateData & _pt)
{
    return _pt.u * visionSettings::pixelToCamResolution;
}

/**
 * @brief VisualLocalization::getRangeFromAngle convert an angle in the FOV to a range index in the lidar
 * @param lrange
 * @return
 */
unsigned int VisualLocalization::getRangeFromAngle(double &_angle)
{
    unsigned int lrange = visionSettings::laser_resolution/2 + visionSettings::laser_resolution/visionSettings::laser_range * _angle;
    return lrange;
}
