#pragma once

#include "ofMain.h"
#include "appUtils.h"
#include "videoProperties.h"

class videoContainer {

public:

	void pause(bool paused);
	void playByTime(int time);
	void init(ofVec2f center, vector<string> elements);
	void update();
	void draw();
	void startAnimation();
	void reorderVideos(appUtils::VideoOrder videoOrder);

	bool readyState = false;
	bool playing = false;

	// video
	ofVideoPlayer actualVideo;

	//frames
	vector <videoProperties> sampleFrames;

	ofVec2f displayCenter;
	int videoPosition = 0;
	int maxHeight = 0;
	int maxWidth = 0;

	float animationStart = 0.f;
	float animationTime = 2.f; // seconds

	float alphaValue = 255;
	float zAnimation = 0;

	appUtils::VideoOrder currentSorting;

private:

	void setVideoProperties(string path);

};

