#pragma once

#include "ofMain.h"

class frame {

public:

	int* pixels;
	int width;
	int height;

	int widthImg;
	int heightImg;

	int minZ = numeric_limits<int>::max();
	int maxZ = numeric_limits<int>::min();

	int minX = numeric_limits<int>::max();
	int minY = numeric_limits<int>::max();

	int maxX = numeric_limits<int>::min();
	int maxY = numeric_limits<int>::min();

	int minXImg = numeric_limits<int>::max();
	int minYImg = numeric_limits<int>::max();

	int maxXImg = numeric_limits<int>::min();
	int maxYImg = numeric_limits<int>::min();

};