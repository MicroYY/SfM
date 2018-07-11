#pragma once

#include <opencv2/opencv.hpp>

struct CameraPose
{
public:
	CameraPose() :K(3, 3), R(3, 3), t(3, 1)
	{
		this->K(0, 0) = 0.; this->K(0, 1) = 0.; this->K(0, 2) = 0.;
		this->K(1, 0) = 0.; this->K(1, 1) = 0.; this->K(1, 2) = 0.;
		this->K(2, 0) = 0.; this->K(2, 1) = 0.; this->K(2, 2) = 0.;
		this->R(0, 0) = 0.; this->R(0, 1) = 0.; this->R(0, 2) = 0.;
		this->R(1, 0) = 0.; this->R(1, 1) = 0.; this->R(1, 2) = 0.;
		this->R(2, 0) = 0.; this->R(2, 1) = 0.; this->R(2, 2) = 0.;
		this->t(0, 0) = 0.; this->t(1, 0) = 0.; this->t(2, 0) = 0.;
		//std::cout << this->K << std::endl;
		//std::cout << this->R << std::endl;
		//std::cout << this->t << std::endl;
	}


public:
	void init_K(double focial_length, double cx, double cy);

	void init_R_t();

	bool is_vaild() const;

public:

	cv::Mat_<double> K;
	cv::Mat_<double> R;
	cv::Mat_<double> t;
	//cv::Vec2d t;
};

inline void
CameraPose::init_K(double focial_length, double cx, double cy)
{
	this->K(0, 0) = focial_length;
	this->K(0, 1) = 0.;
	this->K(0, 2) = cx;
	this->K(1, 0) = 0.;
	this->K(1, 1) = focial_length;
	this->K(1, 2) = cy;
	this->K(2, 0) = 0.;
	this->K(2, 1) = 0.;
	this->K(2, 2) = 1;
	std::cout << "Matrix K =" << std::endl;
	std::cout << this->K << std::endl;
}

inline void
CameraPose::init_R_t()
{
	this->R(0, 0) = 1.;
	this->R(0, 1) = 0.;
	this->R(0, 2) = 0.;
	this->R(1, 0) = 0.;
	this->R(1, 1) = 1.;
	this->R(1, 2) = 0.;
	this->R(2, 0) = 0.;
	this->R(2, 1) = 0.;
	this->R(2, 2) = 1.;
	this->t(0, 0) = 0.;
	this->t(1, 0) = 0.;
	this->t(2, 0) = 0.;
	std::cout << "Matrix R =" << std::endl;
	std::cout << R << std::endl;
	std::cout << "Matrix t=" << std::endl;
	std::cout << t << std::endl;
}

inline bool
CameraPose::is_vaild() const
{
	return this->K(0, 0) != 0.;
}