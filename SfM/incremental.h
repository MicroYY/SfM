#pragma once


#include "scene.h"

class Incremental
{


public:
	void initialize(Scene::ViewList* m_view_list, Scene::TrackList* m_track_list);

	void triangulate_new_tracks(int min_num_views);


private:


private:
	Scene::ViewList* view_list;
	Scene::TrackList* track_list;
};