#pragma once

#include "ofApp.h"

class videoContainer {

public:

	void play();
	void pause();
	void playByTime(int time);
	void init();

	// video
	ofVideoPlayer actualVideo;

	//frames
	vector <ofImage> sampleFrames;

	int videoPosition = 0;

};