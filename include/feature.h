#ifndef SLAM_FEATURE_H_
#define SLAM_FEATURE_H_
#include "include/frame.h"
#include "orb_extractor.h"

namespace slam
{
  class Feature
  {
  public:
    Feature(Frame* frame, const cv::KeyPoint& keypoint, cv::Mat descriptor);
    ~Feature();
    
  private:
    void undistortKeyPoint();
  public:
    Frame* frame_;
    cv::KeyPoint keypoint_;
    cv::KeyPoint undistored_keypoint_;
    cv::Mat descriptor_;
  };
}
#endif