#include <opencv2/opencv.hpp>
#include "include/orb_extractor.h"
#include "include/feature.h"
#include "include/frame.h"
#include "include/pinhole_camera.h"
using namespace slam;
using namespace std;
int main(int argc, char* argv[])
{
  if(argc < 1){
    std::cerr << "Feature detector \nUsage: " << argv[0]  << "[-i|--imgname name]\n" << std::endl;
    return 0;}
   std::string img_name = argv[1];
   std::cout <<"IMG_NAME" <<  img_name << std::endl;
  cv::Mat img(cv::imread(img_name,0));
  assert(img.type() == CV_8UC1 && !img.empty());
  PinholeCamera *cam = new PinholeCamera(640.0, 480.0,
    517.306408, 516.469215, 318.643040, 255.313989,
    0.262383, -0.953104, -0.005358, 0.002628, 1.163314
  );

  ORBextractor *extractor = new ORBextractor();

  Frame frame(cam, img, 0, extractor);
  
  cout << "frame keypoint num " << frame.keypoints_num_ << endl;
  cv::Mat img_rgb(img.size(), CV_8UC3);
  cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
  std::vector<Feature> cv_feats = frame.features_;
  std::for_each(cv_feats.begin(), cv_feats.end(), [&](Feature i)
  {
    cv::circle(img_rgb, i.keypoint_.pt, 4*(i.keypoint_.octave+1), cv::Scalar(0, 255,0), 1);
    cv::circle(img_rgb, i.undistored_keypoint_.pt, 4*(i.undistored_keypoint_.octave+1), cv::Scalar(0,0,255), 1);
  });
  cv::imshow("original", img_rgb);
  //cv::waitKey(0);
  
  cv::Mat un_img;
  
  vector<cv::KeyPoint> cv_keypoints;
  cv::Mat cv_descs;
  cam->undistortImage(img, un_img);
  (*extractor)(un_img, cv_keypoints, cv_descs);
  cout << "Undistort frame keypoint num: "<< cv_keypoints.size() << endl;
  cv::Mat img_opencv(img.size(), CV_8UC3);
  cv::cvtColor(un_img, img_opencv, CV_GRAY2RGB);
  std::for_each(cv_keypoints.begin(), cv_keypoints.end(), [&](cv::KeyPoint i){
 		cv::circle(img_opencv, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
 	});
 	cv::imshow("Undistort", img_opencv);
 	cv::waitKey(0);
	delete cam;
	delete extractor;
  return 0;
}