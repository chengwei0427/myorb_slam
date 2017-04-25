#include "include/feature.h"
#include "include/frame.h"

namespace slam {
  long unsigned int Frame::frame_counter_ = 0;
Frame::Frame(PinholeCamera* cam, const cv::Mat& img, double timestamp, ORBextractor* extractor) : 
id_(frame_counter_++),
cam_(cam),
img_(img),
timestamp_(timestamp),
extractor_(extractor)
{
  levels_num_ = extractor->getLevels();
  scale_factor_ = extractor->getScaleFactor();
  extractORB(img);
}
Frame::~Frame()
{
}
void Frame::extractORB(const cv::Mat& image)
{
  std::vector<cv::KeyPoint> vec_keypoints;
  cv::Mat descriptors;
  (*extractor_)(image, vec_keypoints, descriptors);
  keypoints_num_ = vec_keypoints.size();
  if(keypoints_num_ == 0) return;
  features_.reserve(keypoints_num_);
  for(size_t i = 0; i < vec_keypoints.size();i++)
  {
    Feature fea = Feature(this, vec_keypoints[i], descriptors.row(i));
    features_.push_back(fea);
  }
}

}