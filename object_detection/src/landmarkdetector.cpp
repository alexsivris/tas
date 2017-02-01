#include "landmarkdetector.h"

/**
 * @brief LandmarkDetector::LandmarkDetector create detector, descriptor and matcher objects for later use
 */
LandmarkDetector::LandmarkDetector(){
    detector = SIFT::create(); // SIFT::create() || ORB::create()
    extractor = SIFT::create(); // SIFT::create() || ORB::create()
}

/**
 * @brief LandmarkDetector::computeTemplates detects keypoints and extract descriptors for all templates
 * @param files
 */
void LandmarkDetector::computeTemplates(vector<String> files) {
    for(int i=0; i < files.size(); i++) {
        Mat img = imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
        vector<KeyPoint> keypoints_tmp;
        Mat descriptors_tmp;
        detector->detect(img, keypoints_tmp);
        extractor->compute(img, keypoints_tmp, descriptors_tmp);
        keypoints_vec.push_back(keypoints_tmp);
        descriptors_vec.push_back(descriptors_tmp);
    }
}

/**
 * @brief LandmarkDetector::detectLandmarks detects templates in the current camera image
 * @param files
 */
void LandmarkDetector::detectLandmarks(vector<String> files) {
    BFMatcher matcher(NORM_L2, true);
    Mat img_cam;

    VideoCapture cap;
    cap.open(0);

    if(!cap.isOpened()){
        cout << "Could not open Camera." << endl;
        cap.release();
        return;
    }

    vector<int> foundCount(keypoints_vec.size());

    ros::NodeHandle nh;
    ros::Publisher msg_pub_;
    msg_pub_ = nh.advertise<object_detection::landmark>("landmarkDetection", 1);
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

        // 3) match camera image with all available templates
        for(int i=0; i < descriptors_vec.size(); i++) {

            // get keypoints and descriptors of current template
            vector<KeyPoint> keypoints_template = keypoints_vec[i];
            Mat descriptors_template = descriptors_vec[i];

            // match using the matcher set in the constructor
            vector< DMatch > matches, good_matches;
            matcher.match(descriptors_template, descriptors_cam, matches);

            // get the image points of the matches
            vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
            for( size_t i = 0; i < matches.size(); i++ ) {
                queryIdxs[i] = matches[i].queryIdx;
                trainIdxs[i] = matches[i].trainIdx;
            }
            vector<Point2f> points1; KeyPoint::convert(keypoints_template, points1, queryIdxs);
            vector<Point2f> points2; KeyPoint::convert(keypoints_cam, points2, trainIdxs);

            // calculate homography matrix for the matched image points
            Mat H12 = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold);
            const double det = H12.at<double>(0,0) * H12.at<double>(1,1) - H12.at<double>(1,0) * H12.at<double>(0,1);

            points1.clear();points2.clear();
            KeyPoint::convert(keypoints_template, points1, queryIdxs);
            KeyPoint::convert(keypoints_cam, points2, trainIdxs);

            Mat points1t; // transformed points
            perspectiveTransform(Mat(points1), points1t, H12);

            vector<char> matchesMask( matches.size(), 0 );
            double maxInlierDist = ransacReprojThreshold < 0 ? 3 : ransacReprojThreshold;
            int inliersCnt = 0;
            Point2f center(0,0);
            for( int i = 0; i < points1.size(); i++ ){
                if( norm(points2[i] - points1t.at<Point2f>((int)i,0)) <= maxInlierDist ){ // inlier
                    matchesMask[i] = 1;
                    inliersCnt++;
                    center.x += points2[i].x;
                    center.y += points2[i].y;
                }
            }
            center.x /= inliersCnt;
            center.y /= inliersCnt;

            // check if homography has enough inliers
            if(inliersCnt > 25){
                cout << "Found template nr. " << i << " at Position <" << center.x << "," << center.y << endl;

                // publish data on topic
                object_detection::landmark msg;
                msg.id = i;
                msg.x = center.x;
                msg.y = center.y;
                msg_pub_.publish(msg);
                ros::spinOnce();
             }

#ifdef DRAW_MATCHES // draw matches and center of recognized landmark for debugging if wanted
                Mat img_template = imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
                Mat img_matches;
                circle( img_cam , center, 30.0, Scalar( 0, 0, 255 ) );
                drawMatches( img_template, keypoints_template, img_cam, keypoints_cam, matches, img_matches, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255), matchesMask, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                imshow("Matches", img_matches);
                waitKey(10);
#endif

        }
    }
}

