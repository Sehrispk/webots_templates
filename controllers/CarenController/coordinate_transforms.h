#ifndef CAREN_COORDINATE_TRANSFORMS
#define CAREN_COORDINATE_Transforms

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <cmath>
#include <vector>
#include <map>

//using namespace cv;
using cv::Matx33f, cv::Matx44f, cv::Vec2f, cv::Vec3f, cv::Vec4f;

const Vec3f cube_base(0.0, -0.01+0.11/2, 0.295);
const Vec3f cam_box(0.0, 0.01, 0.84+0.068);

const Vec3f field_size(60, 180, 20);
const float coord_box[3][2] = {{0.1, 0.74}, {-0.8, 0.9}, {-0.02, 0.55}};

cv::Vec<float, 3> caren_to_arm_base(cv::Vec<float, 3> vec)
{
    vec -= cube_base;
    return Vec3f(vec[0], -vec[2], vec[1]);
}

cv::Vec<float, 3> arm_base_to_caren(cv::Vec<float, 3> vec)
{
    Vec3f help_vec(vec[0], vec[2], -vec[1]);
    return help_vec+cube_base;
}

cv::Vec<float, 3> caren_to_cam_base(cv::Vec<float, 3> vec)
{
    vec -= cam_box;
    return vec;
}

cv::Vec<float, 3> cam_base_to_caren(cv::Vec<float, 3> vec)
{
    vec += cam_box;
    return vec;
}

cv::Mat field_3d_to_caren(cv::Mat peak_location)
{
    cv::Mat vec = cv::Mat::zeros(3,1,CV_32F);
    vec.at<float>(0) = peak_location.at<float>(0)/field_size[0] * (coord_box[0][1]-coord_box[0][0]) + coord_box[0][0]; 
    vec.at<float>(1) = peak_location.at<float>(1)/field_size[1] * (coord_box[1][1]-coord_box[1][0]) + coord_box[1][0];
    vec.at<float>(2) = peak_location.at<float>(2)/field_size[2] * (coord_box[2][1]-coord_box[2][0]) + coord_box[2][0]; 

    return vec;
}

cv::Mat caren_to_field_3d(cv::Mat location_3d)
{
    cv::Mat vec = cv::Mat::zeros(3,1,CV_32F);
    vec.at<float>(0) = (location_3d.at<float>(0)-coord_box[0][0]) * field_size[0]/(coord_box[0][1]-coord_box[0][0]);
    vec.at<float>(1) = (location_3d.at<float>(1)-coord_box[1][0]) * field_size[1]/(coord_box[1][1]-coord_box[1][0]);
    vec.at<float>(2) = (location_3d.at<float>(2)-coord_box[2][0]) * field_size[2]/(coord_box[2][1]-coord_box[2][0]);

    return vec;
}

cv::Mat field_2d_to_caren(cv::Mat peak_location)
{
    cv::Mat vec = cv::Mat::zeros(2,1,CV_32F);
    vec.at<float>(0) = peak_location.at<float>(0)/field_size[0] * (coord_box[0][1]-coord_box[0][0]) + coord_box[0][0]; 
    vec.at<float>(1) = peak_location.at<float>(1)/field_size[1] * (coord_box[1][1]-coord_box[1][0]) + coord_box[1][0];

    return vec;
}

cv::Mat caren_to_field_2d(cv::Mat location_2d)
{
    cv::Mat vec = cv::Mat::zeros(2,1,CV_32F);
    vec.at<float>(0) = (location_2d.at<float>(0)-coord_box[0][0]) * field_size[0]/(coord_box[0][1]-coord_box[0][0]);
    vec.at<float>(1) = (location_2d.at<float>(1)-coord_box[1][0]) * field_size[1]/(coord_box[1][1]-coord_box[1][0]);

    return vec;
}

#endif