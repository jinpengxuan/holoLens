#pragma once

#include "ofMain.h"
#include "appUtils.h"
#include "videoProperties.h"

class videoContainer {

public:

	void pause(bool paused);
	void setSpeed(float speed);
	void init(ofVec2f center, vector<string> elements);
	void update();
	void draw();
	void startAnimation();
	void dismissVideo();
	void reorderVideos(appUtils::VideoOrder videoOrder);

	bool playing = false;
	bool dismissing = false;

	string videoName;

	// video
	ofVideoPlayer actualVideo;

	//frames
	vector <videoProperties> sampleFrames;

	appUtils::VideoOrder currentSorting;

private:

	ofVec2f displayCenter;
	int videoPosition = 0;
	int maxHeight = 300;
	int maxWidth = 0;

	float initAnimationStart = 0.f;
	float dismissAnimationStart = 0.f;
	float initAnimationTime = 2.f; // seconds
	float dismissAnimationTime = 1.f; // seconds

	float initAlphaValue = 255;
	float dismissAlphaValue = 255;
	float zAnimation = 0;
	float xAnimation = 0;

	void setVideoProperties(string path);
	void setVisualProperties();

};

