#pragma once

#include "ofMain.h"

class frame {

public:
	//this class serves as the object frame for the detection

	int* pixels;
	int width;
	int height;

	//hand image sizes
	int widthImg;
	int heightImg;

	//nearest point of the frame
	ofVec3f nearPoint = ofVec3f(numeric_limits<int>::max(), numeric_limits<int>::max(), numeric_limits<int>::max());
	
	//maximum z value of the frame
	int maxZ = numeric_limits<int>::min();

	//other borders
	int minX = numeric_limits<int>::max();
	int minY = numeric_limits<int>::max();

	int maxX = numeric_limits<int>::min();
	int maxY = numeric_limits<int>::min();

	int minXImg = numeric_limits<int>::max();
	int minYImg = numeric_limits<int>::max();

	int maxXImg = numeric_limits<int>::min();
	int maxYImg = numeric_limits<int>::min();

};