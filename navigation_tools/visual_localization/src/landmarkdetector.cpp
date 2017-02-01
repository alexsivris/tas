#include "landmarkdetector.h"

/**
 * @brief LandmarkDetector::LandmarkDetector initialization of threshold value for binary representation of image
 * to 128.
 */
LandmarkDetector::LandmarkDetector() :
    m_threshold(128)
{

}

/**
 * @brief LandmarkDetector::getStarId return the resulting template id
 * @return
 */
unsigned int LandmarkDetector::getStarId()
{
    return m_result.id;
}

/**
 * @brief LandmarkDetector::foundTemplate returns true if the template is found in the camera image.
 * To improve the efficiency of the algorithm the search space is reduced by using HSV color masks.
 * @param _img camera source image
 * @param _tpl template to be inspected
 * @return
 */
bool LandmarkDetector::foundTemplate(Mat &_img, LoadedTemplateData &_tpl)
{
    m_camImg = _img;
    int bestIdx = NO_GOOD_MATCH;

    //load templates
    Mat tpl_binary,tpl_hist;
    Mat frame; // rgb
    Mat hsv_frame;
    Mat red_frame;

    int histSize = 256;
    const float range[] = {0,256};
    const float * histRange = { range };
    // get contours in tpl
    cv::threshold(_tpl.src, tpl_binary,m_threshold, 255, cv::THRESH_BINARY);

    // coud be done with switch case..
    int strongContour = 2; // mario 31 red_star 2

    m_result.startype = _tpl.startype;
    cv::findContours(tpl_binary,m_tplContours,cv::noArray(),cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    tpl_binary = cv::Scalar::all(0);
    drawContours(tpl_binary,m_tplContours,strongContour,cv::Scalar::all(255)); //

#ifdef SHOW_IMG
    imshow("Template", tpl_binary);
    waitkey(0);
#endif
    calcHist(&_tpl.src,1,0,Mat(),tpl_hist,1,&histSize,&histRange,true,false);


    // search in img
    cvtColor(m_camImg,hsv_frame,COLOR_BGR2HSV);

    // Color masking
    switch (_tpl.startype )
    {
    case StarType::RED:
        inRange(hsv_frame, Scalar(0,150, 106), Scalar(20, 255, 174), red_frame);
        break;
    case StarType::BLUE:
        inRange(hsv_frame, Scalar(95,141, 0), Scalar(144, 255, 185), red_frame);
        break;
    case StarType::GREEN:
        inRange(hsv_frame, Scalar(37,82, 56), Scalar(76, 207, 255), red_frame);
        break;
    default:
        inRange(hsv_frame, Scalar(0,150, 106), Scalar(20, 255, 174), red_frame);
    }

    Mat er_kernel = Mat::ones(Size(3,3),CV_8UC1);
    // filter noise
    erode(red_frame,red_frame,er_kernel);
    dilate(red_frame,red_frame,Mat());

    vector< vector< cv::Point > > contours; // all contours in img

    cv::cvtColor(m_camImg, m_gray,cv::COLOR_BGR2GRAY);
    cv::threshold(m_gray, m_binary,m_threshold, 255, cv::THRESH_BINARY);

    // use red mask
    red_frame.copyTo(m_binary);
    cv::findContours(m_binary,contours,cv::noArray(),cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);


    int idxBest = findBestMatch(contours, m_tplContours.at(strongContour),tpl_hist);

    m_binary = cv::Scalar::all(0);
    resize(m_binary,m_binary,Size(m_camImg.cols,m_camImg.rows));

    if (idxBest != NO_GOOD_MATCH)
    {
        cv::drawContours(m_binary,contours,idxBest, cv::Scalar::all(255));
        circle(m_binary,m_center,m_radius,Scalar::all(255),3);
        rectangle(m_binary,m_boundRect,Scalar::all(255),3);
        rectangle(m_camImg,m_center,m_center,Scalar(0,255,0),50);
#ifdef DBG_2
        cout << "Best contour size: " << contours.at(idxBest).size()
             << " Center: " << center << " Area: " << contourArea(contours.at(idxBest)) << endl;
        imshow("Output window", m_binary);
        imshow("BGR", m_camImg);
        waitKey(33);
        destroyAllWindows();
#endif
        return true;

    }
    else
    {

#ifdef DBG_2
        imshow("BGR", m_camImg);
        waitKey(33);
        destroyAllWindows();
        imshow("Output window", m_binary);
#endif
        return false;

    }
}

/**
 * @brief LandmarkDetector::locate assign m_result to result of calculation.
 * @return result
 */
TemplateImgData LandmarkDetector::locate()
{
    m_result.u = m_center.x;
    m_result.v = m_center.y;
    return m_result;
}

/**
 * @brief LandmarkDetector::findBestMatch check if contour of template matches to any contour in the masked image.
 * @param _img_cnt all the contours in the masked image
 * @param _tpl_strong_cnt template contour (star shape)
 * @param _tpl_hist histogram of template
 * @return id of best match
 */
int LandmarkDetector::findBestMatch(vector< vector< Point> > &_img_cnt, vector< Point> &_tpl_strong_cnt, Mat &_tpl_hist)
{
    int best=NO_GOOD_MATCH;
    float dist=99;

    for (int i=0; i<_img_cnt.size();i++)
    {
        if (matchShapes(_img_cnt.at(i),_tpl_strong_cnt,CV_CONTOURS_MATCH_I1,2) < dist
                && contourArea(_img_cnt.at(i)) >= 70
                && contourArea(_img_cnt.at(i)) <= 50000 )
        {
            dist = matchShapes(_img_cnt.at(i),_tpl_strong_cnt,CV_CONTOURS_MATCH_I1,2);
            cv::minAreaRect(_img_cnt.at(i));
            minEnclosingCircle(_img_cnt.at(i),m_center,m_radius);
            m_boundRect = boundingRect(_img_cnt.at(i));
            // set after histogram is ok

            Mat mask = Mat::zeros(m_gray.size(),CV_8UC1);
            vector<vector<Point>> mask_hull(1), tpl_hull(1);
            convexHull(_img_cnt.at(i),mask_hull.at(0));
            convexHull(_tpl_strong_cnt,tpl_hull.at(0));
            drawContours(mask,mask_hull,-1,Scalar(255), CV_FILLED);
            Mat masked_gray;
            m_gray.copyTo(masked_gray,mask);
            imshow("Mask",masked_gray);
            best = i;
        }
    }
    return best;
}

