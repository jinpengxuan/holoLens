#include "videoContainer.h"

void videoContainer::init(ofVec2f center, vector<string> elements) {
	// loop through directory and create video visual
	readyState = false;
	sampleFrames.clear();

	vector<string>::iterator iteratorTemp;
	ofImage currentVideoFrame;
	for (iteratorTemp = elements.begin(); iteratorTemp < elements.end(); iteratorTemp++) {
		currentVideoFrame = getSampleFrame((string)*iteratorTemp);
		if (currentVideoFrame.isAllocated())sampleFrames.push_back(currentVideoFrame);
	}

	displayCenter = center;

	actualVideo.closeMovie();

	startAnimation();
}

void videoContainer::draw() {
	if (!readyState)return;

	vector<ofImage>::iterator iteratorFrames;
	vector<ofVec2f>::iterator iteratorDimensions = frameDimensions.begin();
	vector<ofVec3f>::iterator iteratorPositions = framePositions.begin();

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

		ofImage actualFrame = ((ofImage)*iteratorFrames);

		actualFrame.allocate(((ofVec2f)*iteratorDimensions).x, ((ofVec2f)*iteratorDimensions).y, OF_IMAGE_COLOR);
		actualFrame.draw(((ofVec3f)*iteratorPositions).x, ((ofVec3f)*iteratorPositions).y, ((ofVec3f)*iteratorPositions).z-zAnimation, ((ofVec2f)*iteratorDimensions).x, ((ofVec2f)*iteratorDimensions).y);
		iteratorDimensions++;
		iteratorPositions++;
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
	vector<ofImage>::iterator iteratorTemp;
	int count = sampleFrames.size()-1;
	animationStart = ofGetElapsedTimef();
	for (iteratorTemp = sampleFrames.begin(); iteratorTemp < sampleFrames.end(); iteratorTemp++) {

		ofImage actualFrame = ((ofImage)*iteratorTemp);
		int height = actualFrame.getHeight() > maxHeight ? maxHeight : actualFrame.getHeight();
		int width = actualFrame.getHeight() > maxHeight ? (int)(height / (float)maxHeight * actualFrame.getWidth()) : actualFrame.getWidth();
		int difference = (maxHeight - height) / 2;

		float drawX = displayCenter.x - width / 2;
		float drawY = displayCenter.y - height + (difference)+(count * 500);
		float drawZ = (float)(-100 - count * 1000);

		framePositions.push_back(ofVec3f(drawX, drawY, drawZ));
		frameDimensions.push_back(ofVec2f(width, height));
		count--;
	}
	readyState = true;

}

void videoContainer::reorderVideos(appUtils::VideoOrder videoOrder) {
	if (videoOrder == appUtils::VideoOrder::Length) {

	}
}