#ifndef VISUALLOCALIZATION_H
#define VISUALLOCALIZATION_H
#include <iostream>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;
using namespace std;


class VisualLocalization
{
public:
    VisualLocalization();
private:
    void projectPixelToCamera();
    void projectCameraToOrigin();
    void transformQuaternionToR();
    void broadcastCameraFrame();

    MatrixXf *m_K;
    Matrix3f m_R;
    vector<VectorXf> m_vecTemplates;

    Vector3f m_pixelHom;
    tf::Vector3 m_camPositionInBaseLink;
    // tf stuff
    tf::TransformListener m_tfListener;


};

#endif // VISUALLOCALIZATION_H
