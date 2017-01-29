#include "visuallocalization.h"

VisualLocalization::VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names, vector<LoadedTemplateData> &_loaded_tpls) :
    m_nh(_nh), m_gotMap(false), m_bInitLocalization(true),m_loadedTemplates(_loaded_tpls), m_gotPoseArray (false)
{
    cout << "Constructor VL: 1" << endl;

    m_pLandmarkDetector = new LandmarkDetector;

    m_detectedTpl.clear();
    m_landmarks.resize(_lm_names.size());
    for (int i=0; i<_lm_names.size();i++)
    {
        m_landmarks.at(i).src = imread(_lm_names.at(0),1);
        m_landmarks.at(i).id = i;
        m_loadedTemplates.at(i).id = i;
    }


    m_subCamImg = m_nh.subscribe("/usb_cam/image_raw",1,&VisualLocalization::cbCamImg,this);
    m_subTplDetection = m_nh.subscribe("/tpl_detect", 1, &VisualLocalization::cbTplDetect, this); // FROM BENNI
    m_subMap = m_nh.subscribe("/map_image/full",1,&VisualLocalization::cbMap, this);
    m_subParticles = m_nh.subscribe("/particlecloud",1,&VisualLocalization::cbParticles, this);
    m_subAmclPose = m_nh.subscribe("/amcl_pose",1,&VisualLocalization::cbAmclPose,this);
    m_pubPosition = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
}

void VisualLocalization::localize()
{
    //loop
    ros::Rate rt(10);
    while (ros::ok())
    {
        //do localization
        ros::spinOnce();
        rt.sleep();
    }
}


void VisualLocalization::cbAmclPose(const geometry_msgs::PoseWithCovariance::ConstPtr &msg)
{
    m_amclPose.pose = msg->pose;
}

void VisualLocalization::cbCamImg(const sensor_msgs::Image::ConstPtr &_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    m_camImg = cv_ptr->image;


    // call template detector
    for (auto i=0; i<m_loadedTemplates.size(); i++)
    {
        if (m_pLandmarkDetector->foundTemplate(m_camImg,m_loadedTemplates.at(i)))
        {
            unsigned int id = m_pLandmarkDetector->getStarId();
            id = m_loadedTemplates.at(i).id;
            if (!anyOfType(m_detectedTpl,id))
            {
                m_detectedTpl.push_back(m_pLandmarkDetector->locate());
                m_detectedTpl.back().id = id;
            }
        }
    }
    if (m_detectedTpl.size())
    {
        cout << "Found: " << m_detectedTpl.size() << " templates." << endl;
        Point2f pose;
        for (auto i=0;i<m_detectedTpl.size(); i++)
        {
            m_detectedTpl.at(i).theta = convertPointToAngle(m_detectedTpl.at(i));
#ifdef DBG
            cout << "Template " << i << endl <<
                    "----------------------------------------------------" << endl;
            cout << "Coordinates: " << m_detectedTpl.at(i).u <<
                    ", " << m_detectedTpl.at(i).v << endl <<
                    "Distance from car: " << m_detectedTpl.at(i).distance << endl <<
                    "ID: "<< m_detectedTpl.at(i).id << endl <<
                    "Theta: " << m_detectedTpl.at(i).theta  << endl <<
                    "----------------------------------------------------" << endl;
#endif
        }
        if (m_detectedTpl.size() == 3)
        {
            // Set initial pose
            pose = estimatePosition(m_landmarks,
                                            m_detectedTpl.at(0).distance,
                                            m_detectedTpl.at(1).distance,
                                            m_detectedTpl.at(2).distance
                                            );
            m_carPosition.x = pose.x;
            m_carPosition.y = pose.y;
            m_bInitLocalization = false;
        }
        if (m_detectedTpl.size() == 1 && !m_bInitLocalization)
        {
            // simple localization
            pose = simpleLocalization(m_detectedTpl.at(0));

        }
        publishPose(pose);
    }
    m_detectedTpl.clear();
    imshow("Camera img",m_camImg);
    waitKey(3);

}
inline void VisualLocalization::publishPose(Point2f &_pose)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = _pose.x;
    msg.pose.pose.position.y = _pose.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = m_amclPose.pose.orientation;
    msg.pose.covariance[6*0+0] = 0.5 * 0.5;
    msg.pose.covariance[6*1+1] = 0.5 * 0.5;
    msg.pose.covariance[6*3+3] = M_PI/12.0 * M_PI/12.0;

    m_pubPosition.publish(msg);
}

/**
 * @brief VisualLocalization::cbParticles fetch poses from amcl and store them in a member attribute
 * @param _particles
 */
void VisualLocalization::cbParticles(const geometry_msgs::PoseArray::ConstPtr &_particles)
{
    m_particleCloud.poses = _particles->poses;
    m_gotPoseArray = true;
}

/**
 * @brief VisualLocalization::cbLaserScan get distance measure from Lidar
 * @param scan
 */
void VisualLocalization::cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int minIndex = 0;//ceil((min_scan_angle - scan->angle_min) / scan->angle_increment);
    int maxIndex = 719;//floor((max_scan_angle - scan->angle_min) / scan->angle_increment);

    for (int i=0;i<m_detectedTpl.size();i++)
    {
        unsigned int rangeIndex = getRangeFromAngle(m_detectedTpl.at(i).theta);
        if(scan->ranges[rangeIndex]!=INFINITY &&
                rangeIndex <= maxIndex &&
                rangeIndex >= minIndex){
            m_detectedTpl.at(i).distance = scan->ranges[rangeIndex];
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

        // for (auto i; i<m_detectedTpl.size(); i++)
        locateLandmarksInMap();
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        exit(-1);
    }
    ros::spinOnce();
}


/**
 * @brief VisualLocalization::cbTplDetect (Alternative) callback function that gets invoked when 3 templates are detected.
 * @param msg
 */
void VisualLocalization::cbTplDetect(const geometry_msgs::PoseArray::ConstPtr &msg) //TO BE CHANGED
{

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
    // m_bLocalizationMode = true; // ERST TRUE SETZEN WENN 3 TEMPLATES ERKANNT WURDEN, ANSONSTEN FALSE
    // else

    ros::spinOnce();
}
// *************** ** ******************************

inline void VisualLocalization::locateLandmarksInMap()
{
    if (!m_gotMap)
    {
        m_gotMap = true;
        m_pLandmarkMatcher = new LandmarkMatcher(m_mapImg,m_landmarks, m_nh);
        for (auto i=0;i<m_landmarks.size();i++)
            cout << "center of tpl"<< i << " " << m_landmarks.at(i).map_coordinates <<endl;
    }
}

/**
 * @brief VisualLocalization::estimatePosition estimate position of car based on its distance from the landmarks
 * @param _lm
 * @param da
 * @param db
 * @param dc
 * @return Point2f position
 */

inline Point2f VisualLocalization::estimatePosition(vector<LandmarkData> &_lm, float da, float db, float dc)
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

/**
 * @brief VisualLocalization::convertPointToAngle convert a pixel point to an angle in the FOV
 * @param _pt
 * @return
 */
inline double VisualLocalization::convertPointToAngle(TemplateImgData & _pt)
{
    return _pt.u * visionSettings::pixelToCamResolution;
}

/**
 * @brief VisualLocalization::getRangeFromAngle convert an angle in the FOV to a range index in the lidar
 * @param lrange
 * @return
 */
inline unsigned int VisualLocalization::getRangeFromAngle(double &_angle)
{
    unsigned int range = visionSettings::laser_resolution/2 + visionSettings::laser_resolution/visionSettings::laser_range * _angle;
    return range;
}

bool VisualLocalization::anyOfType(vector<TemplateImgData>& _tpl_vec,unsigned int &_id)
{
    for (auto it: _tpl_vec)
    {
        if (it.id == _id)
            return true;
    }
}

Point2f VisualLocalization::simpleLocalization(TemplateImgData &_tpl)
{
    double res=0;
    int t1=0,t2=0;
    int bestId=NO_GOOD_MATCH;
    int lmDist = 9999;
    int closestId=NO_GOOD_MATCH;
    Point2f pose;
    pose.x=0;
    pose.y=0;

    if (m_gotPoseArray)
    {
        pose.x = m_particleCloud.poses.at(0).position.x;
        pose.y = m_particleCloud.poses.at(0).position.y;

        for (auto i=0; i< m_particleCloud.poses.size(); i++)
        {
            //do any of the points lie on the circle
            t1 = m_particleCloud.poses.at(i).position.x - _tpl.x;
            t2 = m_particleCloud.poses.at(i).position.y - _tpl.y;
            res = pow(t1,2) + pow(t2,2) - _tpl.distance;
            if (res < lmDist)
            {
                lmDist = res;
                closestId = i;
            }

            if (res < 0.2) // allow 20cm uncertainty
                bestId = i;
        }
        if (bestId == NO_GOOD_MATCH)
        {
            // take point on circle that is closest to pose array
            double deg = 360;
            double x,y;
            double dist = 99;
            for (auto theta=-deg/2;theta<deg/2;theta++)
            {
                x = _tpl.x + cos(theta)*_tpl.distance;
                y = _tpl.y + sin(theta)*_tpl.distance;
                //distance to BEST pose array estimate
                double d1 = x - m_particleCloud.poses.at(closestId).position.x;
                d1 = pow(d1,2);
                double d2 = y - m_particleCloud.poses.at(closestId).position.y;
                d2 = pow(d2,2);

                double tmp_dist = d1 + d2;
                tmp_dist = sqrt(tmp_dist);
                if (tmp_dist < dist)
                {
                    //dist = tmp_dist;
                    pose.x = x;
                    pose.y = y;
                }
            }

        }
        else
        {
            pose.x = m_particleCloud.poses.at(bestId).position.x;
            pose.y = m_particleCloud.poses.at(bestId).position.y;
        }
    }
    return pose;

}
