#ifndef CAREN_ARM_KINEMATICS
#define CAREN_ARM_KINEMATICS

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <cmath>
#include <vector>
#include <map>

//using namespace cv;
using cv::Matx33f, cv::Matx44f, cv::Vec2f, cv::Vec3f, cv::Vec4f;

// Define arm constants
const float d_bs = 0.31f; // base shoulder
const float d_se = 0.4f; // shoulder elbow
const float d_ew = 0.4f; //elbow wrist
const float d_wf = 0.078f+0.094f; // wrist finger
const float d_cb_y = 0.29f; // cube base y
const float d_cb_zx = 0.05f; // cube base zx
const float Phi_circ = 0.;

// define cam constants
const float d_rc = 0.097; // rotation center camera distance

const float floor_level = -0.02;

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

Matx44f CubeTransformationMatrix(float theta)
{
    return Matx44f( cos(theta), 0, sin(theta), sin(theta)*d_cb_zx,
                    0, 1, 0, -d_cb_y,
                    -sin(theta), 0, cos(theta), cos(theta)*d_cb_zx,
                    0, 0, 0, 1);
}

// Declare function prototype
Vec3f ForwardKinematics(cv::Vec<float, 8> theta)
{
    Vec4f x_e(0,0,0,1);
    for (int i=6; i >=0; i--)
    {
        x_e = TransformationMatrix(i, theta[i])*x_e;
    }
    x_e = CubeTransformationMatrix(theta[7])*x_e;
    Vec3f x(x_e[0], x_e[1], x_e[2]);

    return x;
}

cv::Mat NumericJacobian(Vec3f x, cv::Vec<float, 8> theta)
{
    float dt = 0.000001; //dtheta
    cv::Mat J(3,8,CV_32F, cv::Scalar(0));
    for (int i = 0; i<8; i++)
    {
        cv::Vec<float,8> dtheta;
        dtheta[i] = dt;
        Vec3f dx = (ForwardKinematics(theta+dtheta)-x)/dt;
        J.at<float>(0, i) = dx[0];
        J.at<float>(1, i) = dx[1];
        J.at<float>(2, i) = dx[2];
    }
    return J;
}

cv::Vec<float, 8> ThetaDot(Vec3f x_tar, cv::Vec<float, 8> theta)
{
    float v0 = 0.1;

    Vec3f x = ForwardKinematics(theta);
    Vec3f x_dot = (x_tar-x)/norm(x_tar-x) * v0;
    
    cv::Mat J = NumericJacobian(x, theta);
    cv::Mat theta_dot_m = J.inv(cv::DecompTypes::DECOMP_SVD) * x_dot;
    cv::Vec<float, 8> theta_dot = theta_dot_m.col(0);

    return theta_dot;
}

cv::Vec<float, 3> CameraAnglesToCenterViewPoint( Vec3f rotation_origin, float phi, float theta)
{
    theta += M_PI_2; // theta should be between 0 an Pi/2 (sensor go from 0 (cam pointing down) to -pi/2 (cam_pointing up))
    phi = -phi; // pi/2 should point in positive y direction

    Matx33f R =  RotationMatrix_z(phi)*RotationMatrix_y(theta);
    Vec3f rx(0, 0, d_rc); // default platform vector
    Vec3f ry(0, 1, 0); // default vector in plane of camera platform

    Vec3f cam_pos = R*rx;
    Vec3f cam_dir = -cam_pos.cross(R*ry);
    float table_proj_factor = (floor_level-(rotation_origin[2]+cam_pos[2]))/cam_dir[2]; // factor for projection on the table surface
    Vec3f center_view_point = rotation_origin + cam_pos + table_proj_factor * cam_dir;

    return center_view_point;
}

cv::Vec<float, 2> CenterViewPointToCameraAngles(Vec3f target, Vec3f rotation_origin)
{
    // koordinate transform (camera base coordinates)
    Vec3f x_dif = target-rotation_origin;

    // calculate rotation first
    float phi = atan2(x_dif[1], x_dif[0]); 

    // Now find vector from x_base that is the intersection between the tangent of x_tar-x_base and circle of radius d_rc around 0
    Vec2f M(0, -x_dif[2]);
    Vec2f Q(sqrt(pow(x_dif[0],2)+pow(x_dif[1],2)), 0);
    Vec2f MQ = Q-M;
    float mq = norm(MQ);
    Vec2f help_vec(-Q[1]+M[1], Q[0]-M[0]);
    Vec2f SM = pow(d_rc/mq,2) * MQ + sqrt(pow(mq,2)-pow(d_rc,2))*d_rc/pow(mq,2)*help_vec;
    float theta = acos(SM[1]/norm(SM));

    Vec2f angles(phi, theta);
    return angles;
}

#endif
