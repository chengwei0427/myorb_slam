#ifndef ORB_EXTRACTOR_H_
#define ORB_EXTRACTOR_H_

#include <vector>
#include <list>
#include <opencv/cv.h>

namespace slam
{
class ExtractorNode
{
public:
	ExtractorNode(): is_no_more_(false) {}
	void divideNode(ExtractorNode&n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4);
public:
	std::vector<cv::KeyPoint> vec_keys_;
	cv::Point2i UL_, UR_, BL_, BR_;
	std::list<ExtractorNode>::iterator node_iter_;
	bool is_no_more_;

};
class ORBextractor
{
  public:
	std::vector<cv::Point> pattern_;

	int features_num_;
	float scale_factor_;
	int levels_num_;
	int default_fast_threshold_;
	int min_fast_threshold_;

	std::vector<int> feature_num_per_level_;
	std::vector<int> umax_;
	std::vector<float> vec_scale_factor_;
public:
	ORBextractor(int features_num = 500, float scale_factor = 1.2f, int levels_num = 8,
	             int default_fast_threshold = 20, int min_fast_threshold = 7);
	~ORBextractor() {}
	void operator()(cv::InputArray image, 
	                std::vector<cv::KeyPoint>& keypoints,
	                cv::OutputArray descriptors);
	inline int getLevels()
	{
		return levels_num_;
	}
	inline float getScaleFactor()
	{
		return scale_factor_;
	}
	inline std::vector<float> getScaleFactors()
	{
		return vec_scale_factor_;
	}
protected:
	void computePyramid(cv::Mat image);
	void computeKeyPointsQuadTree(std::vector<std::vector<cv::KeyPoint> >& all_keypoints);
	/*std::vector<cv::KeyPoint> distributeQuadTree(const std::vector<cv::KeyPoint>& vec_to_distribute_keys, const int& min_x,
	        const int&max_x, const int&min_y, const int max_y, const int&feature_num, const int& level);*/
	std::vector<cv::KeyPoint> distributeQuadTree(const std::vector<cv::KeyPoint>& vec_to_distribute_keys, const int &min_x,
	const int &max_x, const int &min_y, const int &max_y, const int &feature_num, const int &level);
public:
	std::vector<cv::Mat> vec_image_pyramid_;

};
}
#endif
