/*
void cv::findContours(
  cv::InputOutputArray    image,         // Input "binary" 8-bit single channel
  cv::OutputArrayOfArrays contours,      // Vector of vectors or points
  cv::OutputArray         hierarchy,     // (optional) topology information
  int                     mode,          // Contour retrieval mode (Figure 14-2)
  int                     method,              // Approximation method
  cv::Point               offset = cv::Point() // (optional) Offset every point
);

void cv::findContours(
  cv::InputOutputArray    image,         // Input "binary" 8-bit single channel
  cv::OutputArrayOfArrays contours,      // Vector of vectors or points
  int                     mode,          // Contour retrieval mode (Figure 14-2)
  int                     method,        // Approximation method
  cv::Point               offset = cv::Point() // (optional) Offset every point
);

void  cv::drawContours(
  cv::InputOutputArray   image,                  // Will draw on input image
  cv::InputArrayOfArrays contours,               // Vector of vectors or points
  int                    contourIdx,             // Contour to draw (-1 is "all")
  const cv::Scalar&      color,                  // Color for contours
  int                    thickness = 1,          // Thickness for contour lines
  int                    lineType  = 8,          // Connectedness ('4' or '8')
  cv::InputArray         hierarchy = noArray(),  // optional (from findContours)
  int                    maxLevel  = INT_MAX,    // Max descent in hierarchy
  cv::Point              offset    = cv::Point() // (optional) Offset all points
)


*/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#define NO_GOOD_MATCH -5

using namespace std;
using namespace cv;

Mat g_gray, g_binary;
int g_thresh = 128;
string g_output_win = "Output contours";
Point2f center;
float radius;

Rect bound_rect;
Mat contourRegion;


// ORB GLOBALS
Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
vector<KeyPoint> tpl_keypoints, scene_keypoints;
Mat tpl_descriptors,scene_descriptors;
Ptr<FeatureDetector> tpl_detector = ORB::create();
Ptr<DescriptorExtractor> tpl_extractor = ORB::create();


Ptr<FeatureDetector> scene_detector = ORB::create();
Ptr<DescriptorExtractor> scene_extractor = ORB::create();


void on_trackbar(int, void*)
{
    cv::threshold(g_gray, g_binary,g_thresh, 255, cv::THRESH_BINARY);
    vector< vector< cv::Point > > contours;
    cv::findContours(
                g_binary,
                contours,
                cv::noArray(),
                cv::RETR_LIST,
                cv::CHAIN_APPROX_SIMPLE);
    g_binary = cv::Scalar::all(0);
    cv::drawContours(g_binary,contours,-1, cv::Scalar::all(255));
    //imshow(g_output_win, g_binary);
}


int findBestMatch(vector< vector< Point> > &_img_cnt, vector< Point> &_tpl_strong_cnt, Mat &_tpl_hist, Mat &_tpl)
{
    int best=NO_GOOD_MATCH; //
    float dist=99;
    // Ptr<ShapeContextDistanceExtractor> mysc = createShapeContextDistanceExtractor();

    ///  Get the mass centers:

    for (int i=0; i<_img_cnt.size();i++)
    {


        if (matchShapes(_img_cnt.at(i),_tpl_strong_cnt,CV_CONTOURS_MATCH_I1,2) < dist  &&
               _img_cnt.at(i).size() >= (_tpl_strong_cnt.size()/5) &&
                        _img_cnt.at(i).size() <= 1300 &&
                contourArea(_img_cnt.at(i)) <= 50000 && contourArea(_img_cnt.at(i)) >= 100 )
        {
            dist = matchShapes(_img_cnt.at(i),_tpl_strong_cnt,CV_CONTOURS_MATCH_I1,2);
            cv::minAreaRect(_img_cnt.at(i));
            minEnclosingCircle(_img_cnt.at(i),center,radius);
            bound_rect = boundingRect(_img_cnt.at(i));
            // set after histogram is ok

            /*// MASK
            Mat mask = Mat::zeros(g_gray.size(),CV_8UC1);
            drawContours(mask,_img_cnt,i,Scalar(255), CV_FILLED);
            Mat masked_gray;
            g_gray.copyTo(masked_gray,mask);
            imshow("Mask",masked_gray);*/
            // MASK
            Mat mask = Mat::zeros(g_gray.size(),CV_8UC1);
            drawContours(mask,_img_cnt,i,Scalar(255), CV_FILLED);
            Mat masked_gray;
            g_gray.copyTo(masked_gray,mask);

            cout << masked_gray.type() << endl;

            // masked histogram
            Mat mask_hist;
            int histSize = 256;
            const float range[] = {0,256};
            const float * histRange = { range };
            calcHist(&masked_gray,1,0,Mat(),mask_hist,1,&histSize,&histRange,true,false);
            normalize( mask_hist, mask_hist, 0, 1, NORM_MINMAX, -1, Mat() );
            normalize( _tpl_hist, _tpl_hist, 0, 1, NORM_MINMAX, -1, Mat() );

            double comp = compareHist(mask_hist,_tpl_hist,CV_COMP_HELLINGER );
            //cout << "Correlation: " << comp << endl;


            scene_detector->detect(masked_gray,scene_keypoints);
            scene_extractor->compute(masked_gray,scene_keypoints,scene_descriptors);

            // Match
            vector< DMatch > matches, good_matches;
            if (tpl_descriptors.rows && scene_descriptors.rows)
            {
                matcher->match(tpl_descriptors, scene_descriptors, matches);
                double max_dist = 0;
                double min_dist = 100;
                for (int i = 0; i < tpl_descriptors.rows; i++){
                    double dist = matches[i].distance;
                    if (dist < min_dist) min_dist = dist;
                    if (dist > max_dist) max_dist = dist;
                }
                for (int i = 0; i < tpl_descriptors.rows; i++){
                    if (matches[i].distance < 2.3 * min_dist){
                        good_matches.push_back(matches[i]);
                    }
                }

                if(good_matches.size() == 0) continue;
                // get the keypoints of the good matches
                std::vector<Point2f> good_pnts_template;
                std::vector<Point2f> good_pnts_cam;
                for (int i = 0; i < good_matches.size(); i++){
                    good_pnts_template.push_back(tpl_keypoints[good_matches[i].queryIdx].pt);
                    good_pnts_cam.push_back(scene_keypoints[good_matches[i].trainIdx].pt);
                }

                cout << "Good matches: " << good_matches.size() << endl;

                // Homography trick
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
                Mat out_img;

                if(good_matches.size() > 3 || det > 0.01 && inliers.size() > 10) {

                    cout << "Found template!!" << endl;
                    drawMatches(_tpl,tpl_keypoints,masked_gray,scene_keypoints,good_matches,out_img);
                    imshow("Mask",out_img);
                    best = i;
                }
                else
                    best = NO_GOOD_MATCH;
                best = i;



                /*if (good_matches.size() > 0 )
                {
                    imshow("Mask",out_img);
                    best = i;
                }
                else
                    best = NO_GOOD_MATCH;*/
                /*if (comp < 0.95)
                {
                    imshow("Mask",out_img);
                    best = i;
                }
                else
                    best = NO_GOOD_MATCH;*/

            }

        }
    }
    return best;
}

/**
 * @brief main This approach combines shape contours with ORB (scale-invariante feature descriptor + detector).
 * @param argc
 * @param argv
 * @return
 */
int main (int argc, char **argv)
{
    namedWindow(g_output_win,1);
    VideoCapture cap(1);
    cout << CV_VERSION << endl;

    cv::createTrackbar(
                "Threshold",
                g_output_win,
                &g_thresh,
                255,
                on_trackbar
                );
    // Load template
    string stpl = "../data/shape_sample/";
    string p1 = stpl + argv[1];
    Mat tpl = imread(p1,CV_LOAD_IMAGE_GRAYSCALE);
    Mat tpl_binary;
    int fixed_thresh = 128;



    Mat zer = Mat(tpl.rows,tpl.cols, CV_8UC1, Scalar(0));
    for (int x=0;x < tpl.cols; x++)
    {
        for (int y=0; y < tpl.rows; y++)
        {
            zer.at<uchar>(y,x) = tpl.at<uchar>(y,x);
        }
    }
    imshow("zr",tpl);

    tpl_detector->detect(zer,tpl_keypoints);
    tpl_extractor->compute(zer,tpl_keypoints,tpl_descriptors);


    cv::threshold(tpl, tpl_binary,g_thresh, 255, cv::THRESH_BINARY);
    vector< vector< cv::Point > > tpl_contours; // all contours in template
    int strongContour = 121; // butterfree 52 jinzo 117 Toad 58 hartley 143 mario 31 magmar 46

    cv::findContours(
                tpl_binary,
                tpl_contours,
                cv::noArray(),
                cv::RETR_LIST,
                cv::CHAIN_APPROX_SIMPLE);
    tpl_binary = cv::Scalar::all(0);
    drawContours(tpl_binary,tpl_contours,strongContour,cv::Scalar::all(255)); //
    imshow("Template", tpl_binary);

    Mat tpl_hist;
    int histSize = 256;
    const float range[] = {0,256};
    const float * histRange = { range };
    calcHist(&tpl,1,0,Mat(),tpl_hist,1,&histSize,&histRange,true,false);


    Mat frame; // rgb
    /* Mat no_bg,hsv_frame;
    Mat red_frame;
    Mat white_frame;
    Ptr<BackgroundSubtractor> pMOG2;
    pMOG2 = createBackgroundSubtractorMOG2();*/


    while (1)
    {
        cap >> frame;
        /*pMOG2->apply(frame,no_bg);
        imshow("asdf",no_bg);*/
        if (frame.empty())
            break;
        /*cvtColor(no_bg,no_bg,COLOR_GRAY2BGR);
        cvtColor(frame,hsv_frame,COLOR_BGR2HSV);
        inRange(hsv_frame, Scalar(140,0, 0), Scalar(179, 255, 255), red_frame);
        imshow("hsv", red_frame);
        // red mask
        /*Mat tmp;

        cvtColor(red_frame,tmp,COLOR_HSV2BGR);
        cvtColor(tmp,red_frame,COLOR_BGR2GRAY);

        threshold(red_frame,red_frame,128,255,THRESH_BINARY);
        imshow("RED", red_frame);*/

        vector< vector< cv::Point > > contours; // all contours in img

        cv::cvtColor(frame, g_gray,cv::COLOR_BGR2GRAY);
        cv::threshold(g_gray, g_binary,g_thresh, 255, cv::THRESH_BINARY);
        /*threshold(g_gray,white_frame,g_thresh,255,THRESH_BINARY);
        imshow("RED", white_frame);
        cvtColor(white_frame,white_frame,COLOR_GRAY2BGR);*/


        cv::findContours(
                    g_binary,
                    contours,
                    cv::noArray(),
                    cv::RETR_LIST,
                    cv::CHAIN_APPROX_SIMPLE);
        int idxBest = findBestMatch(contours, tpl_contours.at(strongContour),tpl_hist, tpl);

        g_binary = cv::Scalar::all(0);
        if (idxBest != NO_GOOD_MATCH)
        {
            cv::drawContours(g_binary,contours,idxBest, cv::Scalar::all(255));
            circle(g_binary,center,radius,Scalar::all(255),3);
            rectangle(g_binary,bound_rect,Scalar::all(255),3);
            rectangle(frame,bound_rect,Scalar(0,0,255),10);
            cout << "Best contour size: " << contours.at(idxBest).size()
                 << " Center: " << center << " Area: " << contourArea(contours.at(idxBest)) << endl;
        }
        imshow(g_output_win, g_binary);


        imshow("BGR", frame);
        waitKey(33);
    }
    destroyAllWindows();
    return 0;
}
