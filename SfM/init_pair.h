#pragma once

#include "camera_pose.h"
#include "scene.h"
#include "correspondence.h"

class InitPair
{


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
	};
	typedef std::vector<CandidatePair> CandidatePairs;

public:
	void initialize(Scene::ViewList const& m_view_list,Scene::TrackList const& m_track_list);

	void compute_pair();

private:
	void init_candidate(CandidatePairs* candidates);

private:
	Scene::ViewList const* view_list;
	Scene::TrackList const* track_list;

	Result init_pair_result;
};