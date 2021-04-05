#ifndef KALMANTEST_H
#define KALMANTEST_H

#include "configure.h"
#include "control.h"

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f predict_point(double _t, Point _p);
    void reset();

    float anti_range = 1.5;

private:
    cv::KalmanFilter KF_;
    Mat measurement_matrix;

    double t = 0.005;
    float x = 640;
    float y = 400;
};

#endif // RM_KALMANFILTER_H
