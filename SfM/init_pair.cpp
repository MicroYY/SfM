#include "init_pair.h"

#include <cmath>



void InitPair::initialize(Scene::ViewList const & m_view_list, Scene::TrackList const & m_track_list)
{
	this->view_list = &m_view_list;
	this->track_list = &m_track_list;
}

void InitPair::compute_pair()
{
	if (this->track_list == nullptr || this->view_list == nullptr)
		throw std::invalid_argument("Null views or tracks");

	std::cout << "Searching for initial pair..." << std::endl;
	this->init_pair_result.view1_id = -1;
	this->init_pair_result.view2_id = -1;

	std::cout << "Initializing candidate pairs..." << std::endl;
	CandidatePairs candidates;
	init_candidate(&candidates);
	std::sort(candidates.rbegin(), candidates.rend());
	std::cout << "Candidate pairs initialized!" << std::endl;

	bool found_pair = false;
	size_t found_pair_id = std::numeric_limits<size_t>::max();

	for (size_t i = 0; i < candidates.size(); i++)
	{
		if (found_pair)
			continue;

		CandidatePair const& candidate = candidates[i];
		size_t num_matches = candidate.matches.size();

		int const min_num_matches = 50;

		if (num_matches < min_num_matches)
		{
			std::cout << "Candidate pair (" << candidate.view1_id << "," << candidate.view2_id
				<< ") rejected! " << num_matches << " of " << min_num_matches << " matches." << std::endl;
			continue;
		}

		std::vector<cv::Point2f> view1_points;
		std::vector<cv::Point2f> view2_points;
		for (size_t i = 0; i < candidate.matches.size(); i++)
		{
			view1_points.push_back(candidate.matches[i].point1);
			view2_points.push_back(candidate.matches[i].point2);
		}

		std::vector<uchar> inliers_mask;
		cv::findHomography(view1_points, view2_points, inliers_mask, CV_RANSAC);
		int num_inliers = 0;
		for (size_t i = 0; i < inliers_mask.size(); i++)
		{
			if (inliers_mask[i])
				num_inliers++;
		}
		float inliers_percentage = static_cast<float>(num_inliers) / num_matches;
		if (inliers_percentage > 0.8)
		{
			std::cout << "Candidate pair (" << candidate.view1_id << "," << candidate.view2_id
				<< ") rejected! " << num_inliers << " of " << min_num_matches << " matches. "
				<< (int)(inliers_percentage * 100) << "%" << std::endl;
			continue;
		}

		CameraPose pose1;
		CameraPose pose2;
		bool const found_pose = compute_pose(candidate, view1_points, view2_points, &pose1, &pose2);
		if (!found_pose)
		{
			continue;
		}
	}
}

void InitPair::init_candidate(CandidatePairs * candidates)
{
	int const num_views = this->view_list->size();

	std::vector<int> candidate_lookup(pow(num_views, 2), -1);
	candidates->reserve(1000);
	for (size_t i = 0; i < this->track_list->size(); i++)
	{
		Scene::Track const& track = this->track_list->at(i);
		for (size_t j = 1; j < track.feature_list.size(); j++)
		{
			for (size_t k = 0; k < j; k++)
			{
				int view1_id = track.feature_list[j].view_id;
				int view2_id = track.feature_list[k].view_id;
				int feature1_id = track.feature_list[j].feature_id;
				int feature2_id = track.feature_list[k].feature_id;

				if (view1_id > view2_id)
				{
					std::swap(view1_id, view2_id);
					std::swap(feature1_id, feature2_id);
				}

				int const look_id = view1_id * num_views + view2_id;
				int pair_id = candidate_lookup[look_id];
				//���Ϊ1����δ��ƥ���
				if (pair_id == -1)
				{
					pair_id = candidates->size();
					candidate_lookup[look_id] = pair_id;
					candidates->push_back(CandidatePair());
					candidates->back().view1_id = view1_id;
					candidates->back().view2_id = view2_id;
				}

				View const& view1 = this->view_list->at(view1_id);
				View const& view2 = this->view_list->at(view2_id);
				//double const x1 = view1.get_keypoints()[feature1_id].pt.x;
				//double const y1 = view1.get_keypoints()[feature1_id].pt.y;
				//double const x2 = view2.get_keypoints()[feature2_id].pt.x;
				//double const y2 = view2.get_keypoints()[feature2_id].pt.y;
				Correspondence2D2D match;
				//match.p1[0] = x1;
				//match.p1[1] = y1;
				//match.p2[0] = x2;
				//match.p2[1] - y2;
				match.point1 = view1.get_keypoints()[feature1_id].pt;
				match.point2 = view2.get_keypoints()[feature2_id].pt;
				candidates->at(pair_id).matches.push_back(match);
			}
		}
	}
}

bool InitPair::compute_pose(CandidatePair const& candidate, std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2,
	CameraPose * pose1, CameraPose * pose2)
{
	cv::Mat_<double> fundamental = cv::findFundamentalMat(points1, points2, CV_FM_LMEDS);
	std::cout << "Pair (" << candidate.view1_id << ","
		<< candidate.view2_id << ")" << " Fundamental: " << std::endl;
	std::cout << fundamental << std::endl << std::endl;

	pose1->init_K(4.0, 0.0, 0.0);
	pose2->init_K(4.0, 0.0, 0.0);
	pose1->init_R_t();

	//std::cout << pose1->K << std::endl;
	cv::Mat_<double> essential;
	cv::transpose(pose2->K, essential);
	//std::cout << essential << std::endl;
	essential = essential * fundamental * pose1->K;
	std::cout << "Matrix essential =" << std::endl;
	std::cout << essential << std::endl;

	cv::Mat_<double> e2 = cv::findEssentialMat(points1, points2, pose1->K, cv::RANSAC);
	//std::cout << e2 << std::endl;
	//cv::Mat_<double> e3 = cv::findEssentialMat(points1, points2, pose1->K, cv::LMEDS);
	//std::cout << e3 << std::endl;

	cv::recoverPose(essential, points1, points2, pose2->K, pose2->R, pose2->t);
	std::cout << pose2->R << std::endl;
	std::cout << pose2->t << std::endl;

	cv::Mat_<double> R(3, 3);
	cv::Mat_<double> t(3, 1);
	int num_inliers = cv::recoverPose(e2, points1, points2, pose2->K, R, t);
	if (num_inliers == 0)
		return false;
	std::cout << R << std::endl;
	std::cout << t << std::endl;
}

double InitPair::angle_between_poses(CandidatePair const & candidate, CameraPose const & pose1, CameraPose const & pose2)
{
	cv::Mat_<double>(3,3) 
}
