#include "visuallocalization.h"

VisualLocalization::VisualLocalization()
{
    m_K = new MatrixXf(3,3);
    m_K->setZero();
    m_pixelHom.setZero();
    cout << (*m_K) * m_pixelHom << endl;
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

void VisualLocalization::projectPixelToCamera()
{

}

void VisualLocalization::transformQuaternionToR()
{
}
