#ifndef SLAM_FRAME_H_
#define SLAM_FRAME_H_
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/concept_check.hpp>
#include <include/pinhole_camera.h>
#include "include/orb_extractor.h"
#include "feature.h"
namespace slam
{
  class Feature;
  typedef std::vector<Feature> Features;
  class Frame
  {
  public:
    Frame(PinholeCamera* cam, const cv::Mat& img, double timestamp, ORBextractor* extractor);
    ~Frame();
  private:
    void extractORB(const cv::Mat& image);
  public:
    static long unsigned int frame_counter_;
    long unsigned int id_;
    double timestamp_;
    PinholeCamera* cam_;
    cv::Mat img_;
    float scale_factor_;
    int levels_num_;
    Features features_;
    int keypoints_num_;
    ORBextractor* extractor_;
  };
}
#endif