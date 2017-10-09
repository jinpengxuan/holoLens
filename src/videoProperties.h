#pragma once

#include "ofMain.h"

class videoProperties {

public:

	//video name
	string name;
	
	//sample image of video
	ofImage sampleFrame;

	//dimension of video
	ofVec2f dimension;

	//position in the 3d space
	ofVec3f position;

	//duration of video
	float duration;

};