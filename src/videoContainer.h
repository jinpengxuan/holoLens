#pragma once

#include "ofMain.h"
#include "applicationProperties.h"
#include "videoProperties.h"

class videoContainer {

public:

	//pause video
	void pause(bool paused);

	//set speed of video
	void setSpeed(float speed);

	//lifecycle functions
	void init(ofVec2f center, vector<string> elements);
	void update();
	void draw();

	//start video initializing animation
	void startAnimation();

	//start dismiss video animation
	void dismissVideo();

	//reorder the video order by passed parameter
	void reorderVideos(applicationProperties::VideoOrder videoOrder);

	//flags
	bool isPaused = true;
	bool dismissing = false;
	float videoControlTime = 0;
	float scaling = 1.0;

	//current video
	string videoName;

	// video
	ofVideoPlayer actualVideo;

	//sample frames of images
	vector <videoProperties> sampleFrames;

	//sorting order of videos
	applicationProperties::VideoOrder currentSorting;

private:

	//center of the display
	ofVec2f displayCenter;

	//sample height
	int maxHeight = 300;

	//animation timer marks
	float initAnimationStart = 0.f;
	float dismissAnimationStart = 0.f;
	float initAnimationTime = 2.f; // seconds
	float dismissAnimationTime = 1.f; // seconds

	//animation transparency and translation values
	float initAlphaValue = 255;
	float dismissAlphaValue = 255;
	float zAnimation = 0;
	float xAnimation = 0;

	//set static properties of sample frames and video
	void setVideoProperties(string path);

	//set positions and dimensions of sample frames and video
	void setVisualProperties();

};

