#pragma once

#include "camera_pose.h"
#include "scene.h"
#include "correspondence.h"

class InitPair
{

public:
	struct Result
	{
		int view1_id;
		int view2_id;
		CameraPose camera1_pose;
		CameraPose camera2_pose;
	};

private:
	struct CandidatePair
	{
		int view1_id;
		int view2_id;
		Correspondences2D2D matches;

		bool operator<(CandidatePair const& other) const
		{
			return this->matches.size() < other.matches.size();
		}
	
	};
	typedef std::vector<CandidatePair> CandidatePairs;

public:
	void initialize(Scene::ViewList const& m_view_list,Scene::TrackList const& m_track_list);

	void compute_pair(Result& result);

private:
	void init_candidate(CandidatePairs* candidates);

	bool compute_pose(CandidatePair const& candidate,std::vector<cv::Point2f> points1, 
		std::vector<cv::Point2f> points2, 
		CameraPose* pose1, CameraPose* pose2);

	double angle_between_poses(CandidatePair const& candidate, CameraPose const& pose1, CameraPose const& pose2);

	float score_for_pair(CandidatePair const& candidate, int num_inliers, double angle);
private:
	Scene::ViewList const* view_list;
	Scene::TrackList const* track_list;

	//Result init_pair_result;
};