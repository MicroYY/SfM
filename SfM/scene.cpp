#include "scene.h"
#include "utils.h"

#include <Windows.h>
#include <iostream>

#include <opencv2/calib3d.hpp>

void Scene::init()
{
	if (this->base_dir == "")
		throw std::exception("invalid dir");

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
			i++;
		}

	} while (FindNextFile(hf, &data) != 0);

	CreateDirectory(features_dir.c_str(), NULL);

}



void Scene::detect_and_match()
{
	for (int i = 0; i < this->view_list.size(); i++)
	{
		view_list[i].detect_features(features_dir);
		//view_list[i].clean();
	}

	for (int i = 0; i < this->view_list.size() - 1; i++)
	{
		View& view1 = view_list[i];
		for (int j = i + 1; j < this->view_list.size(); j++)
		{
			View& view2 = view_list[j];
			two_view_matching(view1, view2);
		}
	}
}

void Scene::clean()
{
	for (int i = 0; i < this->view_list.size(); i++)
		view_list[i].clean();
}

void Scene::generate_track()
{

}

void Scene::two_view_matching(View const & m_view1, View const & m_view2)
{
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
	const cv::Mat& descriptor1 = m_view1.get_descriptor();
	const cv::Mat& descriptor2 = m_view2.get_descriptor();
	const View::Keypoints& keypoints1 = m_view1.get_keypoints();
	const View::Keypoints& keypoints2 = m_view2.get_keypoints();

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
		return;

	std::vector<cv::DMatch> ransac_matches;
	std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;
	for (int i = 0; i < two_way_matches_1_2.size(); i++)
	{
		points1.push_back(keypoints1[two_way_matches_1_2[i].queryIdx].pt);
		points2.push_back(keypoints2[two_way_matches_1_2[i].trainIdx].pt);
	}
	std::vector<uchar> inliersMask(points1.size());
	cv::findFundamentalMat(points1, points2, inliersMask, CV_FM_RANSAC);
	for (int i = 0; i < inliersMask.size(); i++)
	{
		if (inliersMask[i])
			ransac_matches.push_back(two_way_matches_1_2[i]);
	}

	if (ransac_matches.size() < 12)
		return;

	cv::Mat match_result_img;
	const View::ImagePtr& img1 = m_view1.get_image();
	const View::ImagePtr& img2 = m_view2.get_image();
	cv::drawMatches(*img1, keypoints1, *img2, keypoints2, ransac_matches, match_result_img);
	std::string result_path = this->features_dir + "\\" + std::to_string(m_view1.get_view_id())
		+ "with" + std::to_string(m_view2.get_view_id()) + ".jpg";
	cv::imwrite(result_path, match_result_img);

	TwoViewMatching match_result;
	match_result.view_id1 = m_view1.get_view_id();
	match_result.view_id2 = m_view2.get_view_id();
	for (int i = 0; i < ransac_matches.size(); i++)
	{
		CorrespondenceIndex pair;
		pair.first = ransac_matches[i].queryIdx;
		pair.second = ransac_matches[i].trainIdx;
		match_result.matches.push_back(pair);
	}
	this->matching_result.push_back(match_result);
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
	cv::imwrite(str, *this->image_proxy.image);

}

void View::clean()
{
	this->image_proxy.image = nullptr;
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

