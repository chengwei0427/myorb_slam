#include "include/pinhole_camera.h"
namespace slam {
PinholeCamera::PinholeCamera(double width, double height, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3):
width_(width), height_(height),
fx_(fx), fy_(fy),cx_(cx),cy_(cy),
distortion_(fabs(k1) > 0.0000001),
undist_map1_(height_, width_, CV_16SC2),
undist_map2_(height_, width_, CV_16SC2)
{
  d_[0] = k1; d_[1] = k2; d_[2] = p1; d_[3] = p2; d_[4] = k3;
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  dist_coef_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
  cv::initUndistortRectifyMap(cvK_, dist_coef_, cv::Mat_<double>::eye(3,3), cvK_,
    cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_ );
}
PinholeCamera::~PinholeCamera()
{
}
void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rectified)
{
  if(distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
  else
    rectified = raw.clone();
}
}