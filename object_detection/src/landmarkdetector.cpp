#include "landmarkdetector.h"

LandmarkDetector::LandmarkDetector(){
}

void LandmarkDetector::computeTemplates(vector<String> files) {
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> extractor = ORB::create();

    for(int i=0; i < files.size(); i++) {
        Mat img = imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
        vector<KeyPoint> keypoints_tmp;
        Mat descriptors_tmp;
        detector->detect(img, keypoints_tmp);
        extractor->compute(img, keypoints_tmp, descriptors_tmp);
        keypoints.push_back(keypoints_tmp);
        descriptors_vec.push_back(descriptors_tmp);
    }
}

void LandmarkDetector::detectLandmarks() {   
    // ORB to detect and extract features
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> extractor = ORB::create();

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");

    Mat img_cam;


    VideoCapture cap;
    cap.open(0);

    if(!cap.isOpened()){
        cout << "Could not open Camera." << endl;
        cap.release();
        return;
    }

    while(ros::ok){
        // read current image from camera
        cap.read(img_cam);
        cvtColor(img_cam, img_cam, CV_BGR2GRAY);

        // 1) detect keypoints in camera image
        std::vector<KeyPoint> keypoints_cam;
        detector->detect(img_cam, keypoints_cam);

        // 2) extract feature-descriptors from camera image
        Mat descriptors_cam;
        extractor->compute(img_cam, keypoints_cam, descriptors_cam);

        for(int i=0; i < descriptors_vec.size(); i++) {
            Mat descriptors = descriptors_vec[i];
            vector<KeyPoint> keypoints_object = keypoints[i];

            vector< DMatch > matches, good_matches;
            matcher->match(descriptors, descriptors_cam, matches);

            // get only "good" matches
            double max_dist = 0; double min_dist = 100;
            for (int i = 0; i < descriptors.rows; i++){
                double dist = matches[i].distance;
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }
            for (int i = 0; i < descriptors.rows; i++){
                if (matches[i].distance < 3 * min_dist){
                    good_matches.push_back(matches[i]);
                }
            }

            if(good_matches.size() == 0) continue;

            // get the keypoints of the good matches
            std::vector<Point2f> good_pnts_template;
            std::vector<Point2f> good_pnts_cam;
            for (int i = 0; i < good_matches.size(); i++){
                good_pnts_template.push_back(keypoints_object[good_matches[i].queryIdx].pt);
                good_pnts_cam.push_back(keypoints_cam[good_matches[i].trainIdx].pt);
            }

            Mat maskInH;
            Mat H = findHomography(good_pnts_template, good_pnts_cam, CV_RANSAC, 3, maskInH);

            if(H.rows != 3 || H.cols != 3) continue;

            const double det = H.at<double>(0,0) * H.at<double>(1,1) - H.at<double>(1,0) * H.at<double>(0,1);

            // get inliers of homography and calculate first moment of the inlier keypoints
            std::vector<Point2f> inliers;
            Point2f cen(0,0);
            for ( int i=0; i<good_pnts_cam.size(); i++ ){
                if(maskInH.at<float>(i, 0) != 0){
                    cen.x += good_pnts_cam[i].x;
                    cen.y += good_pnts_cam[i].y;
                    inliers.push_back(good_pnts_cam[i]);
                }
            }

            // check if homography is good and has enough inliers
            if(det > 0.1 && inliers.size() > 40) {
                cout << "Found template nr. " << i << endl;
            }

        }
    }
}

//int main(int argc, char **argv)
//{
//    std::cout << "Working with OpenCV Version :" << CV_VERSION << std::endl;

//    ros::init(argc, argv, "landmark_detector");
//    ros::NodeHandle nh;
//    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

//    ros::Rate loop_rate(10);
//    Ptr<Feature2D> orb2 = SURF::create(minHessian);
//    //int minHessian = 400;
//    //Ptr<Feature2D> orb = SURF::create(minHessian);
//    Ptr<ORB> orb = ORB::create();
//    //Ptr<FastFeatureDetector> orb = FastFeatureDetector::create();

//    Mat img_template;
//    Mat img_cam;
//    vector<KeyPoint> keypoints_template;
//    vector<KeyPoint> keypoints_cam;
//    Mat descriptors_template;
//    Mat descriptors_cam;

//    //img_template = imread( "/home/benni/catkin_ws/devel/lib/object_detection/mario.png" , CV_LOAD_IMAGE_GRAYSCALE );
//    img_template = imread( "/home/benni/catkin_ws/devel/lib/object_detection/landmark1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
//    orb->detect( img_template, keypoints_template );
//    orb->compute( img_template, keypoints_template, descriptors_template );

//    VideoCapture cap;
//    cap.open(0);
//    while( ros::ok() ) {
//        cap.read(img_cam);
//        cvtColor(img_cam, img_cam, CV_BGR2GRAY);

//        orb->detect( img_cam, keypoints_cam );
//        orb->compute( img_cam, keypoints_cam, descriptors_cam );


//        std::vector< DMatch > good_matches;

//        /* std::vector<std::vector<cv::DMatch> > matches;
//        cv::BFMatcher matcher;
//        matcher.knnMatch(descriptors_template, descriptors_cam, matches, 2);  // Find two nearest matches*/
//        /*vector<cv::DMatch> good_matches;
//        for (int i = 0; i < matches.size(); ++i)
//        {
//            const float ratio = 0.7; // As in Lowe's paper; can be tuned
//            if (matches[i][0].distance < ratio * matches[i][1].distance)
//            {
//                good_matches.push_back(matches[i][0]);
//            }
//        }*/

//        std::vector< DMatch > matches;
//        FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 24, 2));
//        matcher.match( descriptors_template, descriptors_cam, matches );

//        double max_dist = 0; double min_dist = 100;
//        for( int i = 0; i < descriptors_template.rows; i++ )
//        {
//            double dist = matches[i].distance;
//            if( dist < min_dist ) min_dist = dist;
//            if( dist > max_dist ) max_dist = dist;
//        }

//        for (int i = 0; i < descriptors_template.rows; i++ ){
//            if( matches[i].distance <= max(2*min_dist, 0.02) ) {
//                good_matches.push_back(matches[i]);
//            }
//        }



//         std::cout << "Good matches " <<  good_matches.size() << std::endl;

//           Mat img_matches;
//         drawMatches(img_cam ,keypoints_cam, img_template,keypoints_template,
//                         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//         imshow("Matches", img_matches);
//             waitKey(100);
//    }

//    //drawKeypoints(img_template, keypoints_template, img_template);
//    //imshow("Keypoints Template", img_template);


//    // Read template


//    // Create Descriptor/Extractor Object
//    //int minHessian = 400;
//    //Ptr<Feature2D> orb = SURF::create(minHessian);
//    //Ptr<ORB> orb = ORB::create();
//    //Ptr<FastFeatureDetector> orb = FastFeatureDetector::create();


//    //cap.open(0);
//    //img_template = imread("biere.jpg", CV_LOAD_IMAGE_GRAYSCALE);




//    /*waitKey(2000);

//    while (ros::ok())
//    {
//        std_msgs::String msg;

//        std::stringstream ss;
//        ss << "OpenCV version : " << CV_VERSION;

//        msg.data = ss.str();

//        ROS_INFO("%s", msg.data.c_str());

//        chatter_pub.publish(msg);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }*/


//    return 0;
//}

