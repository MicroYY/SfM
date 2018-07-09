#include "scene.h"
#include "utils.h"

#include <Windows.h>
#include <iostream>

#include <opencv2/calib3d.hpp>

void Scene::init()
{
	if (this->base_dir == "")
		throw std::exception("invalid dir");

	std::cout << "Initializing scene..." << std::endl;

	WIN32_FIND_DATA data;
	HANDLE hf = FindFirstFile((this->base_dir + "\\*").c_str(), &data);
	int i = 0;
	do
	{
		if (!std::strcmp(data.cFileName, "."))
			continue;
		if (!std::strcmp(data.cFileName, ".."))
			continue;

		std::string img_path = this->base_dir + "\\" + data.cFileName;

		std::string ext = right(img_path, 4);
		if (ext == ".png" || ext == ".jpg")
		{
			View single_view(this->base_dir, img_path, i);
			single_view.init();
			this->view_list.push_back(single_view);
			std::cout << "View " << i << " initialized!" << std::endl;
			i++;
		}

	} while (FindNextFile(hf, &data) != 0);

	CreateDirectory(features_dir.c_str(), NULL);

	std::cout << std::endl;
}



void Scene::detect_and_match()
{
	std::cout << "Detecting features..." << std::endl;
	for (int i = 0; i < this->view_list.size(); i++)
	{
		std::cout << "Detecting features of view " << i << std::endl;
		view_list[i].detect_features(features_dir);
		//view_list[i].clean();
	}
	std::cout << std::endl;
	std::cout << "Matching feature..." << std::endl;
	for (int view1_id = 0; view1_id < this->view_list.size() - 1; view1_id++)
	{
		for (int view2_id = view1_id + 1; view2_id < this->view_list.size(); view2_id++)
		{
			//std::cout << "Matching view " << view1_id << " and " << view2_id << "..." << std::endl;
			two_view_matching(view1_id, view2_id);
			//std::cout << std::endl;
		}
	}
	std::cout << std::endl;
}

void Scene::clean()
{
	for (int i = 0; i < this->view_list.size(); i++)
		view_list[i].clean();
}

void Scene::generate_track()
{
	for (int i = 0; i < this->view_list.size(); i++)
	{
		View& view = this->view_list[i];
		view.track_init();
	}

	this->track_list.clear();

	for (int i = 0; i < this->matching_result.size(); i++)
	{
		TwoViewMatching const& tvm = matching_result[i];
		View& view1 = view_list[tvm.view_id1];
		View& view2 = view_list[tvm.view_id2];

		for (int j = 0; j < tvm.matches.size(); j++)
		{
			CorrespondenceIndex idx = tvm.matches[j];

			int const view1_track_id = view1.track_ids[idx.first];
			int const view2_track_id = view2.track_ids[idx.second];
			if (view1_track_id == -1 && view2_track_id == -1)
			{
				view1.track_ids[idx.first] = this->track_list.size();
				view2.track_ids[idx.second] = this->track_list.size();
				this->track_list.push_back(Track());
				this->track_list.back().feature_list.push_back(Feature(tvm.view_id1, idx.first));
				this->track_list.back().feature_list.push_back(Feature(tvm.view_id2, idx.second));
			}
			else if (view1_track_id == -1 && view2_track_id != -1)
			{
				view1.track_ids[idx.first] = view2_track_id;
				this->track_list[view2_track_id].feature_list.push_back(Feature(tvm.view_id1, idx.first));
			}
			else if (view1_track_id != -1 && view2_track_id == -1)
			{
				view2.track_ids[idx.second] = view1_track_id;
				this->track_list[view1_track_id].feature_list.push_back(Feature(tvm.view_id2, idx.second));
			}
			else if (view1_track_id == view2_track_id)
			{
				//std::cout << "123";
			}
			else
			{
				unify_tracks(view1_track_id, view2_track_id);
				//std::cout << view1_track_id << " " << view2_track_id << std::endl;
			}
		}
	}
}

//void Scene::compute_init_pair()
//{
//	this->initial_pair.initialize();
//}

Scene::ViewList & Scene::get_view_list()
{
	return this->view_list;
}

Scene::ViewList const & Scene::get_view_list() const
{
	return this->view_list;
}

Scene::TrackList & Scene::get_track_list()
{
	return this->track_list;
}

Scene::TrackList const & Scene::get_track_list() const
{
	return this->track_list;
}

//Scene::PairwiseMatching & Scene::get_matching()
//{
//	return this->matching_result;
//}
//
//Scene::PairwiseMatching const & Scene::get_matching() const
//{
//	return this->matching_result;
//}

void Scene::two_view_matching(int const m_view1_id, int const m_view2_id)
{
	View const& view1 = view_list[m_view1_id];
	View const& view2 = view_list[m_view2_id];
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
	const cv::Mat& descriptor1 = view1.get_descriptor();
	const cv::Mat& descriptor2 = view2.get_descriptor();
	const View::Keypoints& keypoints1 = view1.get_keypoints();
	const View::Keypoints& keypoints2 = view2.get_keypoints();

	std::vector<std::vector<cv::DMatch>> matches_1_2;
	matcher->knnMatch(descriptor1, descriptor2, matches_1_2, 2);
	std::vector<cv::DMatch> good_matches_1_2;
	for (int i = 0; i < matches_1_2.size(); i++)
	{
		if (matches_1_2[i][0].distance / matches_1_2[i][1].distance < 0.8)
			good_matches_1_2.push_back(matches_1_2[i][0]);
	}

	std::vector<std::vector<cv::DMatch>> matches_2_1;
	matcher->knnMatch(descriptor2, descriptor1, matches_2_1, 2);
	std::vector<cv::DMatch > good_matches_2_1;
	for (int i = 0; i < matches_2_1.size(); i++)
	{
		if (matches_2_1[i][0].distance / matches_2_1[i][1].distance < 0.8)
			good_matches_2_1.push_back(matches_2_1[i][0]);
	}

	std::vector<cv::DMatch> two_way_matches_1_2;
	std::vector<cv::DMatch> two_way_matches_2_1;

	std::vector<int> flag1(matches_1_2.size(), -1);
	std::vector<int> flag2(matches_2_1.size(), -1);

	for (int i = 0; i < good_matches_1_2.size(); i++)
	{
		int query_index = good_matches_1_2[i].queryIdx;
		int train_index = good_matches_1_2[i].trainIdx;
		flag1[query_index] = train_index;
	}
	for (int i = 0; i < good_matches_2_1.size(); i++)
	{
		int query_index = good_matches_2_1[i].queryIdx;
		int train_index = good_matches_2_1[i].trainIdx;
		flag2[query_index] = train_index;
	}


	for (int i = 0; i < flag1.size(); i++)
	{
		if (flag1[i] < 0)
			continue;
		if (flag2[flag1[i]] != i)
			flag1[i] = -1;
	}
	for (int i = 0; i < flag2.size(); i++)
	{
		if (flag2[i] < 0)
			continue;
		if (flag1[flag2[i]] != i)
			flag2[i] = -1;
	}

	for (int i = 0; i < good_matches_1_2.size(); i++)
	{
		int query_index = good_matches_1_2[i].queryIdx;
		if (flag1[query_index] >= 0)
			two_way_matches_1_2.push_back(good_matches_1_2[i]);
	}
	for (int i = 0; i < good_matches_2_1.size(); i++)
	{
		int query_index = good_matches_2_1[i].queryIdx;
		if (flag2[query_index] >= 0)
			two_way_matches_2_1.push_back(good_matches_2_1[i]);
	}

	if (two_way_matches_1_2.size() < 24)
	{
		std::cout << "Pair (" << m_view1_id << "," << m_view2_id 
			<< ") rejected! matches below threshold of 24." << std::endl;
		//std::cout << "Too few matches!" << " View " << m_view1_id << " and " << m_view2_id << " rejected!" << std::endl;
		return;
	}


	std::vector<cv::DMatch> ransac_matches;
	std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;
	for (int i = 0; i < two_way_matches_1_2.size(); i++)
	{
		points1.push_back(keypoints1[two_way_matches_1_2[i].queryIdx].pt);
		points2.push_back(keypoints2[two_way_matches_1_2[i].trainIdx].pt);
	}
	std::vector<uchar> inliers_mask(points1.size());
	cv::findFundamentalMat(points1, points2, inliers_mask, CV_FM_RANSAC,0.5,0.99999);
	for (int i = 0; i < inliers_mask.size(); i++)
	{
		if (inliers_mask[i])
			ransac_matches.push_back(two_way_matches_1_2[i]);
	}

	if (ransac_matches.size() < 15)
	{
		std::cout << "Pair (" << m_view1_id << "," << m_view2_id 
			<< ") rejected! inliers below threshold of 15." << std::endl;
		//std::cout << "Too few matches!" << " View " << m_view1_id << " and " << m_view2_id << " rejected!" << std::endl;
		return;
	}

	cv::Mat match_result_img;
	const View::ImagePtr& img1 = view1.get_image();
	const View::ImagePtr& img2 = view2.get_image();
	cv::drawMatches(*img1, keypoints1, *img2, keypoints2, ransac_matches, match_result_img);
	std::string result_path = this->features_dir + "\\" + std::to_string(m_view1_id)
		+ "-" + std::to_string(m_view2_id) + ".jpg";
	cv::imwrite(result_path, match_result_img);

	TwoViewMatching match_result;
	match_result.view_id1 = m_view1_id;
	match_result.view_id2 = m_view2_id;
	for (int i = 0; i < ransac_matches.size(); i++)
	{
		CorrespondenceIndex pair;
		pair.first = ransac_matches[i].queryIdx;
		pair.second = ransac_matches[i].trainIdx;
		match_result.matches.push_back(pair);
	}
	this->matching_result.push_back(match_result);
	//std::cout << "View " << m_view1_id << " and " << m_view2_id << ": " 
	//	<< ransac_matches.size() << " matches found!" << std::endl;
	std::cout << "Pair (" << m_view1_id << "," << m_view2_id << ") matched! " 
		<< ransac_matches.size() << " inliers!" << std::endl;
}

void Scene::unify_tracks(int view1_track_id, int view2_track_id)
{
	if (track_list[view1_track_id].feature_list.size()
		< track_list[view2_track_id].feature_list.size())
		std::swap(view1_track_id, view2_track_id);

	Track& track1 = track_list[view1_track_id];
	Track& track2 = track_list[view2_track_id];

	for (size_t i = 0; i < track2.feature_list.size(); i++)
	{
		int const view_id = track2.feature_list[i].view_id;
		int const feature_id = track2.feature_list[i].feature_id;
		this->view_list[view_id].track_ids[feature_id] = view1_track_id;
	}
	track1.feature_list.insert(track1.feature_list.end(),
		track2.feature_list.begin(), track2.feature_list.end());
	track2.feature_list = FeatureList();
}

void View::init()
{
	cv::Mat mat = cv::imread(this->image_proxy.img_path);
	while (mat.cols * mat.rows > 6000000)
		cv::resize(mat, mat, cv::Size(mat.cols / 2, mat.rows / 2), 0, 0);

	this->image_proxy.image = std::make_shared<cv::Mat>(mat);
	this->image_proxy.channels = mat.channels();
	this->image_proxy.height = mat.rows;
	this->image_proxy.width = mat.cols;
}

void View::detect_features(std::string const& m_features_dir)
{
	cv::Ptr<cv::DescriptorExtractor> sift = cv::xfeatures2d::SIFT::create(1000);
	sift->detectAndCompute(*this->image_proxy.image, cv::Mat(), this->keypoints, this->descriptor);

	//for (int i = 0; i < keypoints.size(); i++)
	//{
	//	Feature feature;
	//	feature.x = keypoints[i].pt.x;
	//	feature.y = keypoints[i].pt.y;
	//	cv::Mat temp = descriptor.rowRange(i, i + 1);
	//	cv::transpose(temp, temp);
	//
	//	cv::cv2eigen(temp, feature.descriptor);
	//	Eigen::VectorXf v = feature.descriptor;
	//
	//
	//	this->feature_list.push_back(feature);
	//
	//}


	for each (cv::KeyPoint var in keypoints)
	{
		cv::circle(*this->image_proxy.image, cv::Point(var.pt.x, var.pt.y), 2, cv::Scalar(255, 0, 0));
	}
	std::string str = m_features_dir + "\\" + std::to_string(this->view_id) + ".jpg";
	//cv::imwrite(str, *this->image_proxy.image);

}

void View::clean()
{
	this->image_proxy.image = nullptr;
}

void View::track_init()
{
	this->track_ids.resize(this->keypoints.size(), -1);
}

cv::Mat const & View::get_descriptor() const
{
	return this->descriptor;
}

cv::Mat View::get_descriptor()
{
	return this->descriptor;
}

View::Keypoints const & View::get_keypoints() const
{
	return this->keypoints;
}

View::Keypoints View::get_keypoints()
{
	return this->keypoints;
}

View::ImagePtr const & View::get_image() const
{
	return this->image_proxy.image;
}

View::ImagePtr View::get_image()
{
	return this->image_proxy.image;
}

int View::get_view_id()
{
	return view_id;
}

int const & View::get_view_id() const
{
	return view_id;
}

void View::set_pose(CameraPose const& m_pose)
{
	this->pose = m_pose;
}



