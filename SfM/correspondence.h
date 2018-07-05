#pragma once

#include <vector>

struct Correspondence2D2D
{
	double p1[2];
	double p2[2];
};

typedef std::vector<Correspondence2D2D> Correspondences2D2D;

typedef std::pair<int, int> CorrespondenceIndex;
typedef std::vector<CorrespondenceIndex> CorrespondenceIndices;