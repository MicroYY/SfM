#include "incremental.h"





void Incremental::initialize(Scene::ViewList* m_view_list, Scene::TrackList* m_track_list)
{
	this->view_list = m_view_list;
	this->track_list = m_track_list;


	if (this->view_list->empty())
		throw std::invalid_argument("No views given");

	//检查是否至少有两个相机被初始化
	size_t num_valid_cameras = 0;
	for (size_t i = 0; i < this->view_list->size(); i++)
	{
		CameraPose& pose = this->view_list->at(i).get_pose();
		if (pose.is_vaild())
			num_valid_cameras++;
	}
	if (num_valid_cameras < 2)
		throw std::invalid_argument("Two or more valid cameras required");

	//将 track positions 设置为invalid
	for (size_t i = 0; i < m_track_list->size(); i++)
	{
		Scene::Track& track = m_track_list->at(i);
		track.invalidate();
	}

}

void Incremental::triangulate_new_tracks(int min_num_views)
{


	size_t initial_tracks_size = this->track_list->size();

	for (size_t i = 0; i < this->track_list->size(); i++)
	{
		Scene::Track const& track = this->track_list->at(i);
		if (track.valid)
			continue;
		
		std::vector<cv::Point2f> points2D;
		std::vector<CameraPose const*> poses;
		std::vector<int> view_ids;
		std::vector<int> feature_ids;
		for (size_t j = 0; j < track.feature_list.size(); j++)
		{
			int const view_id = track.feature_list[j].view_id;
			CameraPose const& pose = this->view_list->at(view_id).get_pose();
			if (!pose.is_vaild())
				continue;
			View const& view = this->view_list->at(view_id);
			int const feature_id = track.feature_list[j].feature_id;
			points2D.push_back(view.get_keypoints()[feature_id].pt);
			poses.push_back(&view.get_pose());
			view_ids.push_back(view_id);
			feature_ids.push_back(feature_id);
		}

		if (poses.size() < min_num_views)
			continue;


	}
}
