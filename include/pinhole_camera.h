#ifndef SLAM_PINHOLE_CAMERA_H_
#define SLAM_PINHOLE_CAMERA_H_

#include <opencv2/opencv.hpp>
#include <boost/concept_check.hpp>
namespace slam
{
  class  PinholeCamera
  {
  public:
    PinholeCamera(double width, double height, double fx, double fy, double cx, double cy,
		  double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
    ~PinholeCamera();
    inline int width() const { return width_;}
    inline int height() const { return height_;}
    virtual double getFocalLength() const { return fabs(fx_);}
    
    void undistortImage(const cv::Mat& raw, cv::Mat& rectified);
  
    inline double fx() const {return fx_;}
    inline double fy() const {return fy_;}
    inline double cx() const {return cx_;}
    inline double cy() const {return cy_;}
    
    inline double invfx() const
    {
      if(fx_ == 0)
	return 0;
      return 1.0/fx_;
    }
    inline double invfy() const
    {
      if(fy_ == 0) 
	return 0;
      return 1.0/fy_;
    }
    inline cv::Mat cvK() const { return cvK_;}
    inline cv::Mat distCoef() const { return dist_coef_;}
  private:
    int width_;
    int height_;
    double fx_, fy_;
    double cx_, cy_;
    bool distortion_;
    double d_[5];
    cv::Mat cvK_, dist_coef_;
    cv::Mat undist_map1_, undist_map2_;
  };
}
#endif