#pragma once
#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
//#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


#include <Eigen/Eigen>


class View;

class Scene
{
public:
	Scene() {}
	Scene(std::string const& m_base_path) 
		:base_dir(m_base_path), 
		features_dir(m_base_path + "\\features")
	{
		//feature_dir
	}

	

	void init();

	void detect_and_match();

private:
	std::string base_dir;
	std::string features_dir;
	std::vector<View> view_list;

};


class View
{
public:
	typedef std::shared_ptr<cv::Mat> image_ptr;


	struct ImageProxy
	{
		std::string img_path;
		image_ptr image;
		int32_t width = 0;
		int32_t height = 0;
		int32_t channels = 0;

		ImageProxy(std::string const& m_img_path) :img_path(m_img_path) {}

	};
	struct Feature
	{
		Eigen::VectorXf descriptor;
		float x;
		float y;

	};


	View(std::string const& m_base_dir, std::string const& m_img_path, int m_view_id)
		:view_id(m_view_id), base_dir(m_base_dir), image_proxy(m_img_path)
	{}

	void set_img_path();

	void init();

	void detect_features(std::string const& m_features_dir);

private:
	int view_id;
	std::string base_dir;

	ImageProxy image_proxy;
	std::vector<Feature> feature_list;
};

