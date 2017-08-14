#include "videoContainer.h"

void videoContainer::init(ofVec2f center, vector<string> elements) {
	// loop through directory and create video visual
	sampleFrames.clear();

	std::vector<string>::iterator iteratorTemp;
	ofImage currentVideoFrame;
	for (iteratorTemp = elements.begin(); iteratorTemp < elements.end(); iteratorTemp++) {
		currentVideoFrame = getSampleFrame((string)*iteratorTemp);
		if (currentVideoFrame.isAllocated())sampleFrames.push_back(currentVideoFrame);
	}

	displayCenter = center;

	actualVideo.closeMovie();
}

void videoContainer::draw() {
	if (!readyState)return;

	std::vector<ofImage>::iterator iteratorFrames;
	std::vector<ofVec2f>::iterator iteratorDimensions = frameDimensions.begin();
	std::vector<ofVec3f>::iterator iteratorPositions = framePositions.begin();
	int count = sampleFrames.size();
	for (iteratorFrames = sampleFrames.begin(); iteratorFrames < sampleFrames.end(); iteratorFrames++) {
		iteratorDimensions++;
		iteratorPositions++;

		ofImage actualFrame = ((ofImage)*iteratorFrames);

		actualFrame.allocate(((ofVec2f)*iteratorDimensions).x, ((ofVec2f)*iteratorDimensions).y, OF_IMAGE_COLOR);
		actualFrame.draw(((ofVec3f)*iteratorPositions).x, ((ofVec3f)*iteratorPositions).y, ((ofVec3f)*iteratorPositions).z, ((ofVec2f)*iteratorDimensions).x, ((ofVec2f)*iteratorDimensions).y);
		count--;
	}
}

void videoContainer::play() {

}

void videoContainer::pause() {

}

void videoContainer::playByTime(int time) {

}

ofImage videoContainer::getSampleFrame(std::string path) {
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
	std::vector<ofImage>::iterator iteratorTemp;
	int count = sampleFrames.size();
	for (iteratorTemp = sampleFrames.begin(); iteratorTemp < sampleFrames.end(); iteratorTemp++) {

		ofImage actualFrame = ((ofImage)*iteratorTemp);
		int height = actualFrame.getHeight() > maxHeight ? maxHeight : actualFrame.getHeight();
		int width = actualFrame.getHeight() > maxHeight ? (int)(height / (float)maxHeight * actualFrame.getWidth()) : actualFrame.getWidth();
		int difference = (maxHeight - height) / 2;

		float drawX = displayCenter.x - width / 2;
		float drawY = displayCenter.y - height / 2 + (difference)+(count * 500);
		float drawZ = (float)(-400 - count * 1000);

		framePositions.push_back(ofVec3f(drawX, drawY, drawZ));
		frameDimensions.push_back(ofVec2f(width, height));
		count--;
	}

	readyState = true;
}