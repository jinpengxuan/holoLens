#pragma once

#include "ofMain.h"
#include "applicationProperties.h"
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
	void reorderVideos(applicationProperties::VideoOrder videoOrder);

	bool isPaused = true;
	bool dismissing = false;
	float videoControlTime = 0;
	float scaling = 1.0;

	string videoName;

	// video
	ofVideoPlayer actualVideo;

	//frames
	vector <videoProperties> sampleFrames;

	applicationProperties::VideoOrder currentSorting;

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

