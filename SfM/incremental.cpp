#include "incremental.h"





void Incremental::initialize(Scene::ViewList* m_view_list, Scene::TrackList* m_track_list)
{
	this->view_list = m_view_list;
	this->track_list = m_track_list;

}

void Incremental::triangulate_new_tracks(int min_num_views)
{


	size_t initial_tracks_size = this->track_list->size();
	for (size_t i = 0; i < this->track_list->size(); i++)
	{
		Scene::Track const& track = this->track_list->at(i);
		
	}
}
