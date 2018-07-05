
#include "scene.h"
#include "init_pair.h"




int main(int argc, char** argv)
{


	Scene my_scene(argv[1]);
	my_scene.init();
	
	my_scene.detect_and_match();
	my_scene.generate_track();

	InitPair init_pair;

	Scene::ViewList const& view_list = my_scene.get_view_list();
	//Scene::PairwiseMatching const& matching_result = my_scene.get_matching();
	Scene::TrackList const& track_list = my_scene.get_track_list();
	init_pair.initialize(view_list, track_list);
	init_pair.compute_pair();

	return 0;
}

