#pragma once

#include "ofMain.h"

class videoContainer {

public:

	void play();
	void pause();
	void playByTime(int time);
	void init();
	void draw(ofVec2f center);

	// video
	ofVideoPlayer actualVideo;

	//frames
	vector <ofImage> sampleFrames;

	int videoPosition = 0;

private:

	ofImage getSampleFrame(std::string path);

};

