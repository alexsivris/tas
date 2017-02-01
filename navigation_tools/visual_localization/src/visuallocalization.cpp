#include "visuallocalization.h"

/**
 * @brief VisualLocalization::VisualLocalization In the constructor the landmark images are loaded and
 * saved into a vector. Each landmark and each template gets a unique id with which it can be conveniently
 * specified. Optionally the map is loaded from an image file (JPG/PNG) instead of fetching it
 * from hector_compressed_map_transport (recommended). Moreover, the subscribers and publishers are prepared.
 * @param _nh
 * @param _lm_names
 * @param _loaded_tpls
 * @param _use_map_img
 * @param _map_path
 */
VisualLocalization::VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names, vector<LoadedTemplateData> &_loaded_tpls , bool _use_map_img, string _map_path) :
    m_nh(_nh), m_gotMap(false), m_bInitLocalization(true),m_loadedTemplates(_loaded_tpls), m_gotPoseArray (false)
{
    cout << "Constructor VL: 1" << endl;

    m_pLandmarkDetector = new LandmarkDetector;

    m_detectedTpl.clear();
    m_landmarks.resize(_lm_names.size());
    for (int i=0; i<_lm_names.size();i++)
    {
        m_landmarks.at(i).src = imread(_lm_names.at(i),0);
        m_landmarks.at(i).id = i;
        m_loadedTemplates.at(i).id = i;
    }

    m_subCamImg = m_nh.subscribe("/usb_cam/image_raw",1,&VisualLocalization::cbCamImg,this);
    m_subTplDetection = m_nh.subscribe("/tpl_detect", 1, &VisualLocalization::cbTplDetect, this); // FROM BENNI
    m_subParticles = m_nh.subscribe("/particlecloud",1,&VisualLocalization::cbParticles, this);
    m_subAmclPose = m_nh.subscribe("/amcl_pose",1,&VisualLocalization::cbAmclPose,this);
    m_subScan = m_nh.subscribe("/scan",1,&VisualLocalization::cbLaserScan,this);
    m_pubPosition = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
    m_pubLandmarks = m_nh.advertise<sensor_msgs::PointCloud>("/tpl_in_map",1);
    if (!_use_map_img)
    {
        m_subMap = m_nh.subscribe("/map_image/full",1,&VisualLocalization::cbMap, this);
    }
    else
    {
        m_mapImg = imread(_map_path,CV_LOAD_IMAGE_GRAYSCALE);
        locateLandmarksInMap();
    }
}

/**
 * @brief VisualLocalization::~VisualLocalization some garbage collection.
 */
VisualLocalization::~VisualLocalization()
{
    delete m_pLandmarkDetector;
    delete m_pLandmarkMatcher;
}

/**
 * @brief VisualLocalization::localize loop of process. When the initial localization is successful the orientation of the
 * nav_origin frame is calculated and finally the frame is published using a tf broadcaster.
 */
void VisualLocalization::localize()
{
    //loop
    ros::Rate rt(10);
    while (ros::ok())
    {
        //do localization
        if (!m_bInitLocalization)
        {
            calculateOriginOrientation();
            broadcastOriginFrame();
        }
        ros::spinOnce();
        rt.sleep();
    }
}

/**
 * @brief VisualLocalization::cbAmclPose callback function of "/amcl_pose". Member attribute m_amclPose is set to the msg pose
 * @param msg
 */
void VisualLocalization::cbAmclPose(const geometry_msgs::PoseWithCovariance::ConstPtr &msg)
{
    m_amclPose.pose = msg->pose;
}

/**
 * @brief VisualLocalization::cbCamImg callback function of camera image. The camera image is published from the usb_cam package.
 * Every image frame is searched for templates and if any are found, their position is saved in the m_detectedTpl vector.
 * The triangluation approach is only executed when three landmarks are detected. If only one is detected a simpler localization
 * approach is done.
 * @param _img
 */
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
            m_carOrigin.x = pose.x;
            m_carOrigin.y = pose.y;
            calculateOriginOrientation();
            m_bInitLocalization = false;
        }
        if (m_detectedTpl.size() == 1 && !m_bInitLocalization) // do this only if origin has been set
        {
            // simple localization
            pose = simpleLocalization(m_detectedTpl.at(0));
        }
        publishPose(pose);
    }

    imshow("Camera img",m_camImg);
    waitKey(3);
    ros::spinOnce();

}

/**
 * @brief VisualLocalization::publishPose publish the position of the car into the /initialpose topic. In case of initial
 * localization, the orientation is also set for the nav_origin frame.
 * @param _pose
 */
inline void VisualLocalization::publishPose(Point2f &_pose)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = _pose.x;
    msg.pose.pose.position.y = _pose.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = m_amclPose.pose.orientation;
    if (m_detectedTpl.size() == 3)
    {
        tf::Quaternion quat;
        quat.setRPY(0,0,m_carOrigin.theta);
        geometry_msgs::Quaternion q;
        tf::quaternionTFToMsg(quat,q);
        msg.pose.pose.orientation = q;
    }
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
 * @brief VisualLocalization::cbLaserScan get distance measure from laser scanner
 * @param scan
 */
void VisualLocalization::cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int minIndex = 0;
    int maxIndex = 719;

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

/**
 * @brief VisualLocalization::cbMap callback function when message is received from hector_compressed_map_transport package.
 * As soon as a message is received the templates are localized within the image of the map.
 * @param msg
 */
void VisualLocalization::cbMap(const sensor_msgs::ImageConstPtr& msg)
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


/**
 * @brief VisualLocalization::cbTplDetect (Alternative) callback function that gets called when 3 templates are detected. The method
 * is not used during the current execution of the node. The purpose of this method is to act as an interface between my node and Benni's
 * object_detection node that finds the templates by SIFT.
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

    ros::spinOnce();
}

/**
 * @brief VisualLocalization::locateLandmarksInMap instantiate LandmarkMatcher object, which calculates landmark map coordinates
 */
inline void VisualLocalization::locateLandmarksInMap()
{
    if (!m_gotMap)
    {
        m_gotMap = true;
        m_pLandmarkMatcher = new LandmarkMatcher(m_mapImg,m_landmarks, m_pubLandmarks);
        for (auto i=0;i<m_landmarks.size();i++)
            cout << "center of tpl" << i << " " << m_landmarks.at(i).map_coordinates <<endl;
    }
}

/**
 * @brief VisualLocalization::estimatePosition estimate position of car based on its distance from the landmarks. For the three landmarks
 * (call them A,B,C) we know the distances and so we can write a system of three non-linear equations:
 * I: (dist_a)^2 = (x-x_a)^2 + (x-x_b)^2
 * II: (dist_b)^2 = (x-x_b)^2 + (x-x_b)^2
 * III: (dist_c)^2 = (x-x_c)^2 + (x-x_c)^2
 * the solution of this system is evaluated in this method.
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

/**
 * @brief VisualLocalization::broadcastOriginFrame broadcast the nav_origin frame using a tf Transform and getting the position/angle
 * from the estimated car position.
 */
void VisualLocalization::broadcastOriginFrame()
{

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(m_carOrigin.x,m_carOrigin.y,0.0));
    tf::Quaternion q;
    q.setRPY(0.0,0.0,m_carOrigin.theta);
    transform.setRotation(q);
    m_originBroadcaster.sendTransform(tf::StampedTransform(transform,
                                                           ros::Time::now(),
                                                           "map",
                                                           "/nav_origin"));
}

/**
 * @brief VisualLocalization::calculateOriginOrientation The orientation of the origin is set accordin to the vector pointing from the
 * car position to the centroid of the three landmarks. As the centroid is not in the middle of the lane, this orientation is not 100%
 * accurate, but AMCL can correct small deviations over time.
 */
void VisualLocalization::calculateOriginOrientation()
{
    Point2f centroid;
    centroid.x = m_landmarks.at(0).map_coordinates.x +
            m_landmarks.at(1).map_coordinates.x +
            m_landmarks.at(2).map_coordinates.x;
    centroid.x /= 3;
    centroid.y = m_landmarks.at(0).map_coordinates.y +
            m_landmarks.at(1).map_coordinates.y +
            m_landmarks.at(2).map_coordinates.y;
    centroid.y /= 3;

    double tmp;
    tmp = m_carOrigin.x - centroid.x;
    tmp /= (m_carOrigin.y - centroid.y);
    m_carOrigin.theta = atan(tmp);
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

/**
 * @brief VisualLocalization::anyOfType search the template vector for a given id
 * @param _tpl_vec
 * @param _id
 * @return
 */
bool VisualLocalization::anyOfType(vector<TemplateImgData>& _tpl_vec,unsigned int &_id)
{
    for (auto it: _tpl_vec)
    {
        if (it.id == _id)
            return true;
    }
}

/**
 * @brief VisualLocalization::simpleLocalization Simple localization approach. The two options for the pose estimate are:
 * 1.) the point in the particle cloud (from AMCL) which lies within 20cm on the circle around the landmark with radius equal
 * to the distance of the landmark from the car.
 * 2.) the point on the circle which is closest to the particle cloud.
 * @param _tpl
 * @return
 */
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
