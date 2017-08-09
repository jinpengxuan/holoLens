#include "videoContainer.h"

void videoContainer::init() {
	// loop through directory and create video visual
	sampleFrames.resize(4);

	ofImage currentVideoFrame = getSampleFrame("vids/4.avi");
	if(currentVideoFrame.isAllocated())sampleFrames.push_back(currentVideoFrame);

	currentVideoFrame = getSampleFrame("vids/2.mkv");
	if (currentVideoFrame.isAllocated())sampleFrames.push_back(currentVideoFrame);

	currentVideoFrame = getSampleFrame("vids/3.mkv");
	if (currentVideoFrame.isAllocated())sampleFrames.push_back(currentVideoFrame);

	currentVideoFrame = getSampleFrame("vids/1.mkv");
	if (currentVideoFrame.isAllocated())sampleFrames.push_back(currentVideoFrame);

	actualVideo.closeMovie();
}

void videoContainer::draw(ofVec2f center) {
	std::vector<ofImage>::iterator iteratorTemp;
	int count = sampleFrames.size();
	for (iteratorTemp = sampleFrames.begin(); iteratorTemp < sampleFrames.end(); iteratorTemp++) {
		
		ofImage actualFrame = ((ofImage)*iteratorTemp);
		int height = actualFrame.getHeight() > maxHeight ? maxHeight : actualFrame.getHeight();

		float drawX = center.x - actualFrame.getWidth() / 2;
		float drawY = center.y - maxHeight / 2;
		actualFrame.draw(drawX, drawY, (float)(-10 - count * 500), actualFrame.getWidth(), height);
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

	actualVideo.play();
	float pct = 0.5f;
	actualVideo.setPosition(pct);

	return ofImage(actualVideo.getPixels());
}