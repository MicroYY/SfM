
#include "scene.h"
#include "init_pair.h"
#include "incremental.h"



int main(int argc, char** argv)
{


	Scene my_scene(argv[1]);
	my_scene.init();
	
	my_scene.detect_and_match();
	my_scene.generate_track();

	InitPair init_pair;

	Scene::ViewList & view_list = my_scene.get_view_list();
	//Scene::PairwiseMatching const& matching_result = my_scene.get_matching();
	Scene::TrackList & track_list = my_scene.get_track_list();

	InitPair::Result init_result;
	init_pair.initialize(view_list, track_list);
	init_pair.compute_pair(init_result);

	view_list[init_result.view1_id].set_pose(init_result.camera1_pose);
	view_list[init_result.view2_id].set_pose(init_result.camera2_pose);

	Incremental incremental;
	incremental.initialize(&view_list, &track_list);
	incremental.triangulate_new_tracks(2);

	return 0;
}