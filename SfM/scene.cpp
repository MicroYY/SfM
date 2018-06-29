#include "scene.h"

#include <Windows.h>
#include <iostream>

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
		View single_view(this->base_dir,img_path,i);
		single_view.init();
		this->view_list.push_back(single_view);			
		i++;
	} while (FindNextFile(hf, &data) != 0);

	CreateDirectory(features_dir.c_str(), NULL);
	//std::cout << this->view_list.size();
}



void Scene::detect_and_match()
{
	for (int i = 0; i < this->view_list.size(); i++)
	{
		view_list[i].detect_features(features_dir);
	}
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
	cv::Ptr<cv::DescriptorExtractor> sift = cv::xfeatures2d::SIFT::create();
	std::vector<cv::KeyPoint> keypoints;
	//sift->detect(img, keypoints);
	cv::Mat descriptor;
	sift->detectAndCompute(*this->image_proxy.image, cv::Mat(), keypoints, descriptor);
	for each (cv::KeyPoint var in keypoints)
	{
		cv::circle(*this->image_proxy.image, cv::Point(var.pt.x, var.pt.y), 2, cv::Scalar(255, 0, 0));
	}
	std::string str = m_features_dir + "\\" + std::to_string(this->view_id) + ".jpg";
	cv::imwrite(str, *this->image_proxy.image);

}
