
#include "scene.h"





int main(int argc, char** argv)
{


	Scene my_scene(argv[1]);
	my_scene.init();
	
	my_scene.detect_and_match();
	my_scene.generate_track();
	return 0;
}

