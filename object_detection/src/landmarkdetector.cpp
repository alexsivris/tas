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

void LandmarkDetector::detectLandmarks(vector<String> files) {
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

    int foundId;
    int foundCnt;

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
                if (matches[i].distance < 2.3 * min_dist){
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

            //cout << good_matches.size() << endl;


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
            //cout << "Det " << det << " inliers " << inliers.size() << " good " << good_matches.size() << endl;
            if(good_matches.size() > 15 || det > 0.1 && inliers.size() > 40) {
                if(foundId != i){
                    foundCnt = 0;
                    foundId = i;
                }
                foundCnt ++;
                if(foundCnt >= 2) {
                    cout << "Found template nr. " << i << endl;
                }

                // publish data on topic
                object_detection::landmark msg;
                msg.id = i;
                msg.x = cen.x;
                msg.y = cen.y;
                msg_pub_.publish(msg);
                ros::spinOnce();
             }

            if(false) {
                Mat img_template = imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
                Mat img_matches;
                drawMatches( img_template, keypoints_object, img_cam, keypoints_cam,
                             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                             maskInH, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                imshow("Matches", img_matches);
                waitKey(100);
            }

        }
    }
}

