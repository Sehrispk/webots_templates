#ifndef CAREN_ARM_KINEMATICS
#define CAREN_ARM_KINEMATICS

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <cmath>
#include <vector>
#include <map>

//using namespace cv;
using cv::Matx33f, cv::Matx44f, cv::Vec2f, cv::Vec3f, cv::Vec4f;

// Define constants
const float d_bs = 0.31f;
const float d_se = 0.4f;
const float d_ew = 0.4f;
const float d_wf = 0.078f;
const float Phi_circ = 0.;

const std::vector<float> alphas = {-M_PI_2,M_PI_2,M_PI_2,-M_PI_2,-M_PI_2,M_PI_2,0};
const std::vector<float> ds = {d_bs,0,d_se,0,d_ew,0,d_wf};
const std::vector<float> as = {0,0,0,0,0,0,0};

// Define helper functions
Matx33f RotationMatrix_x(float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return cv::Matx33f(1, 0, 0, 0, c, -s, 0, s, c);
}

Matx33f RotationMatrix_y(float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return cv::Matx33f(c, 0, s, 0, 1, 0, -s, 0, c);
}

Matx33f RotationMatrix_z(float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return Matx33f(c, -s, 0, s, c, 0, 0, 0, 1);
}

Matx44f TransformationMatrix(int i, float theta) {
    float alpha = alphas[i];
    float d = ds[i];
    float a = as[i];
    float c = cos(theta);
    float s = sin(theta);
    float ca = cos(alpha);
    float sa = sin(alpha);
    return Matx44f(c, -s * ca, s * sa, a * c,
                   s, c * ca, -c * sa, a * s,
                   0, sa, ca, d,
                   0, 0, 0, 1);
}

// Declare function prototype
Vec3f ForwardKinematics(cv::Vec<float, 7> theta)
{
    Vec4f x_e(0,0,0,1);
    for (int i=6; i >=0; i--)
    {
        x_e = TransformationMatrix(i, theta[i])*x_e;
    }
    Vec3f x(x_e[0], x_e[1], x_e[2]);

    return x;
}

cv::Mat NumericJacobian(Vec3f x, cv::Vec<float, 7> theta)
{
    float dt = 0.000001; //dtheta
    cv::Mat J(3,7,CV_32F, cv::Scalar(0));
    for (int i = 0; i<7; i++)
    {
        cv::Vec<float,7> dtheta;
        dtheta[i] = dt;
        Vec3f dx = (ForwardKinematics(theta+dtheta)-x)/dt;
        J.at<float>(0, i) = dx[0];
        J.at<float>(1, i) = dx[1];
        J.at<float>(2, i) = dx[2];
    }
    return J;
}

cv::Mat BoundaryRepellor(cv::Vec<float, 7> theta, cv::Vec<float, 7> min_theta, cv::Vec<float, 7> max_theta)
{
    float beta = 1000.;
    float sigma = .2;
    cv::Mat F(7,1,CV_32F, cv::Scalar(0));
    for (int i = 0; i < 7; i++)
    {
        float f_min = beta* (theta[i]-min_theta[i])*exp(-(theta[i]-min_theta[i])*(theta[i]-min_theta[i])/(2*sigma*sigma));
        float f_max = beta* (theta[i]-max_theta[i])*exp(-(theta[i]-max_theta[i])*(theta[i]-max_theta[i])/(2*sigma*sigma));
        F.at<float>(i,0) = f_min+f_max;
    }
    return F;
}

cv::Mat TargetAttractor(cv::Mat theta)
{
    float lambda = 0.5;
    cv::Mat F(7,1,CV_32F, cv::Scalar(0));
    for (int i=0; i<7; i++)
    {
        F.at<float>(i,0) = -lambda*sin(theta.at<float>(i,0));
    }
    return F;
}

cv::Vec<float, 7> ThetaDot(Vec3f x_tar, cv::Vec<float, 7> theta)
{
    float v0 = 0.1;

    Vec3f x = ForwardKinematics(theta);
    Vec3f x_dot = (x_tar-x)/norm(x_tar-x) * v0;
    
    cv::Mat J = NumericJacobian(x, theta);
    cv::Mat theta_dot_m = J.inv(cv::DecompTypes::DECOMP_SVD) * x_dot;
    cv::Vec<float, 7> theta_dot = theta_dot_m.col(0);

    return theta_dot;
}

#endif
