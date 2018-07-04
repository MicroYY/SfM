#pragma once
#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
//#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>


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

	void clean();

	void generate_track();


	typedef std::pair<int, int> CorrespondenceIndex;
	typedef std::vector<CorrespondenceIndex> CorrespondenceIndices;
	struct TwoViewMatching
	{
		int view_id1;
		int view_id2;
		CorrespondenceIndices matches;

	};
	typedef std::vector<TwoViewMatching> PairwiseMatching;

	struct Feature
	{
		int view_id;
		int feature_id;

		Feature(int m_view_id,int m_feature_id):view_id(m_view_id),feature_id(m_feature_id) {}
	};
	typedef std::vector<Feature> FeatureList;

	struct Track
	{
		FeatureList feature_list;
	};
	typedef std::vector<Track> TrackList;

private:
	void two_view_matching(View const& m_view1, View const& m_view2);

	void unify_tracks(int view1_track_id, int view2_track_id);

private:
	std::string base_dir;
	std::string features_dir;
	std::vector<View> view_list;

	PairwiseMatching matching_result;

	TrackList track_list;
};


class View
{
public:
	typedef std::shared_ptr<cv::Mat> ImagePtr;
	typedef std::vector<cv::KeyPoint> Keypoints;

	struct ImageProxy
	{
		std::string img_path;
		ImagePtr image;
		int32_t width = 0;
		int32_t height = 0;
		int32_t channels = 0;

		ImageProxy(std::string const& m_img_path) :img_path(m_img_path) {}
	};



	View(std::string const& m_base_dir, std::string const& m_img_path, int m_view_id)
		:view_id(m_view_id), base_dir(m_base_dir), image_proxy(m_img_path)
	{}

	void set_img_path();

	void init();

	void detect_features(std::string const& m_features_dir);

	void clean();

	void track_init();

	cv::Mat const& get_descriptor() const;
	cv::Mat get_descriptor();
	Keypoints const& get_keypoints() const;
	Keypoints get_keypoints();
	ImagePtr const& get_image() const;
	ImagePtr get_image();

	int get_view_id();
	int const& get_view_id() const;

public:
	std::vector<int> track_ids;
private:
	int view_id;
	std::string base_dir;
	ImageProxy image_proxy;


	cv::Mat descriptor;
	Keypoints keypoints;


};

