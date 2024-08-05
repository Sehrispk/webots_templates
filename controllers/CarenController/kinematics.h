#ifndef CAREN_ARM_KINEMATICS
#define CAREN_ARM_KINEMATICS

//#include <opencv2/core.hpp>
//#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <map>


//using namespace cv;
using cv::Matx33f; 
using cv::Matx44f; 
using cv::Vec2f;
using cv::Vec3f;
using cv::Vec4f;

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

const float floor_level = 0.0435;

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

cv::Vec<float, 3> extractAngularVelocity(Matx33f S)
{
    //extracts angular velocity from skew symmetric angular velocity matrix. (Equal to the dual of the matrix)
    Vec3f omega(S(2, 1), S(0, 2), S(1, 0));
    return omega;
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

Matx33f getRotationMatrix(cv::Vec<float, 8> theta)
{
    // calculates rotation matrix of eef for certain joint positions
    Matx44f transformation = TransformationMatrix(6, theta[6]);
    for (int i = 5; i >= 0; i--)
    {
        transformation = TransformationMatrix(i, theta[i]) * transformation;
    }
    transformation = CubeTransformationMatrix(theta[7]) * transformation;

    Matx33f rotation;
    for (int i = 0; i < 3; i++)
    {
        rotation(0, i) = transformation(0, i);
        rotation(1, i) = transformation(1, i);
        rotation(2, i) = transformation(2, i);
    }


    return rotation;
}

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

cv::Mat LinearNumericJacobian(Vec3f x, cv::Vec<float, 8> theta)
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

cv::Mat RotationalNumericJacobian(cv::Vec<float, 8> theta, cv::Mat J_linear)
{
    float dt = 0.000001; //dtheta
    cv::Mat J(6, 8, CV_32F, cv::Scalar(0));

    Matx33f R0 = getRotationMatrix(theta);

    for (int i = 0; i < 8; i++)
    { 
        cv::Vec<float, 8> dtheta;
        dtheta[i] = dt;

        Matx33f R1 = getRotationMatrix(theta + dtheta);

        Matx33f dR = R1 * R0.inv() / dt;

        Vec3f omega = extractAngularVelocity(dR);

        J.at<float>(0, i) = J_linear.at<float>(0, i);
        J.at<float>(1, i) = J_linear.at<float>(1, i);
        J.at<float>(2, i) = J_linear.at<float>(2, i);
        J.at<float>(3, i) = omega[0];
        J.at<float>(4, i) = omega[1];
        J.at<float>(5, i) = omega[2];
    }
    return J;
}

cv::Vec<float, 8> ThetaDot_lin(Vec3f x_tar, cv::Vec<float, 8> theta)
{
    // Inverse differential kinematics for simple eef position control
    float v0 = 0.1;

    Vec3f x = ForwardKinematics(theta);
    float l = sqrt( pow(x_tar(0)-x(0),2) + pow(x_tar(1)-x(1),2) + pow(x_tar(2)-x(2),2) );
    Vec3f x_dot = (x_tar-x)/l * v0;
    
    cv::Mat J = LinearNumericJacobian(x, theta);
    cv::Mat theta_dot_m = J.inv(cv::DecompTypes::DECOMP_SVD) * x_dot;
    cv::Vec<float, 8> theta_dot = theta_dot_m.col(0);

    return theta_dot;
}

cv::Vec<float, 8> ThetaDot_rot(Matx33f R_tar, cv::Vec<float, 8> theta)
{
    float v0 = 0.5;

    // get linear component of Jacobian
    Vec3f p_cur = ForwardKinematics(theta);
    cv::Mat J_linear = LinearNumericJacobian(p_cur, theta);

    // get full rotational and linear component of Jacobian
    cv::Mat J(6, 8, CV_32F, cv::Scalar(0));
    J = RotationalNumericJacobian(theta, J_linear);

    // get desired rotational velocity
    cv::Vec<float, 6> p_dot_rot(0,0,0,0,0,0);
    Matx33f R_cur = getRotationMatrix(theta);

    Matx33f dR = R_tar * R_cur.inv();
    Vec3f omega_tar = extractAngularVelocity(dR);
    omega_tar = omega_tar / sqrt(pow(omega_tar(0), 2) + pow(omega_tar(1), 2) + pow(omega_tar(2), 2)) * v0;
    p_dot_rot(3) = omega_tar(0);
    p_dot_rot(4) = omega_tar(1);
    p_dot_rot(5) = omega_tar(2);

    // Velocity component for rotation in null space of jacobian (no impact on position);
    cv::Mat q_dot_rot = J.inv(cv::DecompTypes::DECOMP_SVD) * p_dot_rot;
    cv::Vec<float, 8> joint_velocities_rotation_component = q_dot_rot.col(0);

    return joint_velocities_rotation_component;
}

cv::Vec<float, 3> CameraAnglesToCenterViewPoint( Vec3f rotation_origin, float phi, float theta, float distance)
{
    theta += M_PI_2; // theta should be between 0 an Pi/2 (sensor go from 0 (cam pointing down) to -pi/2 (cam_pointing up))
    phi = -phi; // pi/2 should point in positive y direction
    if (isfinite(distance)==0)
    {
        distance = 2;
    }

    Matx33f R =  RotationMatrix_z(phi)*RotationMatrix_y(theta);
    Vec3f rx(0, 0, d_rc); // default platform vector
    Vec3f ry(0, 1, 0); // default vector in plane of camera platform

    Vec3f cam_pos = R*rx;
    Vec3f cam_dir = -cam_pos.cross(R*ry);
    float table_proj_factor = (floor_level-(rotation_origin[2]+cam_pos[2]))/cam_dir[2]; // factor for projection on the table surface
    Vec3f center_view_point = rotation_origin + cam_pos + table_proj_factor * cam_dir;// 

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
    float mq = sqrt( pow(MQ(0),2) + pow(MQ(1),2) );
    Vec2f help_vec(-Q[1]+M[1], Q[0]-M[0]);
    Vec2f SM = pow(d_rc/mq,2) * MQ + sqrt(pow(mq,2)-pow(d_rc,2))*d_rc/pow(mq,2)*help_vec;
    float sm = sqrt( pow(SM(0),2) + pow(SM(1),2) );
    float theta = acos(SM[1]/sm);

    Vec2f angles(phi, theta);
    return angles;
}

cv::Vec<float, 2> HeadCenteredAnglesToJointAngles(Vec2f head_angles, float distance)
{
    // calculate allocentric position in cartesian coordinates
    Vec3f p( sin(head_angles(1)) * cos(head_angles(0)) , sin(head_angles(1)) * sin(head_angles(0)), cos(head_angles(1)) );
    p = distance * p;

    // calculate pan
    float phi = atan2(p[1], p[0]);

    // Now find vector from that is the intersection between the tangent of x_dir and circle of radius d_rc around 0 (Jans Code)
    Vec2f Q(sqrt(pow(p[0], 2) + pow(p[1], 2)), p[2]);
    float q = sqrt( pow(Q(0),2) + pow(Q(1),2) );
    Vec2f help_vec(-Q[1], Q[0]);
    Vec2f SM = pow(d_rc / q, 2) * Q + sqrt(pow(q, 2) - pow(d_rc, 2)) * d_rc / pow(q, 2) * help_vec;

    // calculate tilt
    float theta = 0;
    if (SM[0] >= 0)
    {
        theta = acos(SM[1] / d_rc);
    }
    else if (SM[0] < 0)
    {
        theta = -acos(SM[1] / d_rc);
    }

    Vec2f joint_angles(phi, theta);

    //std::cout << p << std::endl;
    //std::cout << SM << std::endl;
    //std::cout << phi << std::endl;
    //std::cout << theta << std::endl;

    return joint_angles;
}

cv::Vec<float, 2> JointAnglesToHeadCenteredAngles(Vec2f joint_angles, float distance)
{
    Matx33f R = RotationMatrix_z(joint_angles(0)) * RotationMatrix_y(joint_angles(1));
    Vec3f rx(0, 0, d_rc); // default platform vector
    Vec3f ry(0, 1, 0); // default vector in plane of camera platform

    // head direction and position of camera
    Vec3f cam_pos = R * rx;
    Vec3f cam_dir = -cam_pos.cross(R * ry);
    float l = sqrt( pow(cam_dir(0),2) + pow(cam_dir(1),2) + pow(cam_dir(2),2) );
    cam_dir = cam_dir/l;;

    Vec3f p = cam_pos + distance * cam_dir; // head centered position of focus of camera
    float pd = sqrt( pow(p(0),2) + pow(p(1),2) + pow(p(2),2) );

    // calculate head centered angles
    float phi = atan2(p(1),p(0));
    float theta = acos(p(2)/pd);
    if (!isfinite(theta))
    {
        theta = acos(cam_dir(2));
        std::cout << "ALAARM!!! EIN BERECHNETER HEAD CEENTERED WINKEL WAR INF!!!!!" << std::endl;
    }
    Vec2f head_angles(phi, theta);
    //std::cout << cam_pos << std::endl;
    //std::cout << cam_dir << std::endl;
    //std::cout << p << std::endl;
    //std::cout << head_angles/M_PI*180 << std::endl;

    return head_angles;
}

#endif
