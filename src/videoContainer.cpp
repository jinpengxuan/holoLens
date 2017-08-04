#include "videoContainer.h"

void videoContainer::init() {
	// loop through directory and create video visual
	sampleFrames.resize(2);
	actualVideo.load("vids/1.mkv");
	actualVideo.play();
	float pct = 0.5f;
	actualVideo.setPosition(pct);
	actualVideo.getCurrentFrame();
}

void videoContainer::play() {

}

void videoContainer::pause() {

}

void videoContainer::playByTime(int time) {

}