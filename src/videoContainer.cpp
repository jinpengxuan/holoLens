#include "videoContainer.h"

void videoContainer::init(ofVec2f center, vector<string> elements) {
	// loop through directory and create video visual
	readyState = false;
	sampleFrames.clear();

	vector<string>::iterator iteratorTemp;
	ofImage currentVideoFrame;
	for (iteratorTemp = elements.begin(); iteratorTemp < elements.end(); iteratorTemp++) {
		currentVideoFrame = getSampleFrame((string)*iteratorTemp);
		videoProperties currentProperty = videoProperties();
		currentProperty.sampleFrame = currentVideoFrame;
		if (currentVideoFrame.isAllocated())sampleFrames.push_back(currentProperty);
	}

	displayCenter = center;

	actualVideo.closeMovie();

	startAnimation();
}

void videoContainer::draw() {
	if (!readyState)return;

	vector<videoProperties>::iterator iteratorFrames;

	float alphaValue = 255;
	float zAnimation = 0;
	if ((ofGetElapsedTimef() - animationStart) <= animationTime) {
		float timeParameter = (ofGetElapsedTimef() - animationStart) / animationTime;
		alphaValue = timeParameter * 255;
		zAnimation = (1 - timeParameter) * 50;
	}
	ofEnableAlphaBlending();
	ofSetColor(255, 255, 255, alphaValue);
	for (iteratorFrames = sampleFrames.begin(); iteratorFrames < sampleFrames.end(); iteratorFrames++) {

		ofImage actualFrame = ((videoProperties)*iteratorFrames).sampleFrame;

		actualFrame.allocate(((videoProperties)*iteratorFrames).dimension.x, ((videoProperties)*iteratorFrames).dimension.y, OF_IMAGE_COLOR);
		actualFrame.draw(((videoProperties)*iteratorFrames).position.x, ((videoProperties)*iteratorFrames).position.y, ((videoProperties)*iteratorFrames).position.z-zAnimation, ((videoProperties)*iteratorFrames).dimension.x, ((videoProperties)*iteratorFrames).dimension.y);
	}
	ofDisableAlphaBlending();
}

void videoContainer::play() {

}

void videoContainer::pause() {

}

void videoContainer::playByTime(int time) {

}

ofImage videoContainer::getSampleFrame(string path) {
	actualVideo.load(path);
	if (!actualVideo.isLoaded())return ofImage();

	actualVideo.setVolume(0);
	actualVideo.play();
	float pct = 0.5f;
	actualVideo.setPosition(pct);
	ofImage returnImage = ofImage(actualVideo.getPixels());
	if (returnImage.getHeight() > maxHeight) maxHeight = returnImage.getHeight();

	return returnImage;
}

void videoContainer::startAnimation() {
	int count = sampleFrames.size()-1;
	animationStart = ofGetElapsedTimef();
	for (videoProperties& iteratorTemp : sampleFrames) {

		ofImage actualFrame = iteratorTemp.sampleFrame;
		int height = actualFrame.getHeight();
		int width = actualFrame.getWidth();
		int difference = (maxHeight - height) / 2;

		float drawX = displayCenter.x - width / 2;
		float drawY = displayCenter.y - height + (difference)+(count * 500);
		float drawZ = (float)(-100 - count * 1000);

		iteratorTemp.position = ofVec3f(drawX, drawY, drawZ);
		iteratorTemp.dimension = ofVec2f(width, height);
		count--;
	}
	readyState = true;

}

void videoContainer::reorderVideos(appUtils::VideoOrder videoOrder) {
	if (videoOrder == appUtils::VideoOrder::Length) {

	}
}