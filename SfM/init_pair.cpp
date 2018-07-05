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

	CandidatePairs candidates;
	init_candidate(&candidates);
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
				//如果为1，则还未有匹配对
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
				double const x1 = view1.get_keypoints()[feature1_id].pt.x;
				double const y1 = view1.get_keypoints()[feature1_id].pt.y;
				double const x2 = view2.get_keypoints()[feature2_id].pt.x;
				double const y2 = view2.get_keypoints()[feature2_id].pt.y;
				Correspondence2D2D match;
				match.p1[0] = x1;
				match.p1[1] = y1;
				match.p2[0] = x2;
				match.p2[1] - y2;
				candidates->at(pair_id).matches.push_back(match);
			}
		}
	}
}
