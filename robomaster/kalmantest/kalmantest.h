#ifndef KALMANTEST_H
#define KALMANTEST_H

#include "configure.h"
#include "control.h"
#define ANTI_RANGE 1.01//指数增长的底数
#define A 1.0e-8//给加速度a一个限定值(-A,A)之间
#define MNC 1e-5//测量协方差矩阵R，更大会有更慢的回归
#define DEAD_BAND 0
#define SIZE_X 960
#define SIZE_Y 480
class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f point_Predict(double g_runtime,Point2d current_point);

private:
    Mat measurement_img;//测量矩阵
    cv::KalmanFilter kalman_img;
    Point2f last_point;
//    Point2f last_predic_point;
    double runtime=(1e-2) + 0.005666666f;
    double last_v=0;
    int n=2;

};

#endif // KALMANTEST_H