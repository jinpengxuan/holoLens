#include "videoContainer.h"

bool sortVecByLengthAsc(videoProperties i, videoProperties j) { return (i.duration < j.duration); }
bool sortVecBySizeAsc(videoProperties i, videoProperties j) { return ((i.dimension.x + i.dimension.y) < (j.dimension.x + j.dimension.y)); }

bool sortVecByLengthDesc(videoProperties i, videoProperties j) { return (i.duration > j.duration); }
bool sortVecBySizeDesc(videoProperties i, videoProperties j) { return ((i.dimension.x + i.dimension.y) > (j.dimension.x + j.dimension.y)); }

void videoContainer::init(ofVec2f center, vector<string> elements) {
	// loop through directory and create video visual
	readyState = false;
	sampleFrames.clear();

	vector<string>::iterator iteratorTemp;
	ofImage currentVideoFrame;
	for (iteratorTemp = elements.begin(); iteratorTemp < elements.end(); iteratorTemp++) {
		setVideoProperties((string)*iteratorTemp);
	}

	displayCenter = center;

	actualVideo.closeMovie();

	startAnimation();
}

void videoContainer::draw() {
	if (!readyState)return;

	float alphaValue = 255;
	float zAnimation = 0;
	if ((ofGetElapsedTimef() - animationStart) <= animationTime) {
		float timeParameter = (ofGetElapsedTimef() - animationStart) / animationTime;
		alphaValue = timeParameter * 255;
		zAnimation = (1 - timeParameter) * 50;
	}
	ofEnableAlphaBlending();
	ofSetColor(255, 255, 255, alphaValue);
	for (videoProperties& iteratorTemp : sampleFrames) {

		ofImage actualFrame = iteratorTemp.sampleFrame;

		actualFrame.allocate(iteratorTemp.dimension.x, iteratorTemp.dimension.y, OF_IMAGE_COLOR);
		actualFrame.draw(iteratorTemp.position.x, iteratorTemp.position.y, iteratorTemp.position.z-zAnimation, iteratorTemp.dimension.x, iteratorTemp.dimension.y);
	}
	ofDisableAlphaBlending();
}

void videoContainer::play() {

}

void videoContainer::pause() {

}

void videoContainer::playByTime(int time) {

}

void videoContainer::setVideoProperties(string path) {
	actualVideo.load(path);
	if (!actualVideo.isLoaded())return;
	videoProperties currentProperty = videoProperties();

	currentProperty.duration = actualVideo.getDuration();
	actualVideo.setVolume(0);
	actualVideo.play();
	float pct = 0.5f;
	actualVideo.setPosition(pct);
	ofImage sampleImage = ofImage(actualVideo.getPixels());
	if (sampleImage.getHeight() > maxHeight) maxHeight = sampleImage.getHeight();
	if (sampleImage.getWidth() > maxWidth) maxWidth = sampleImage.getWidth();

	currentProperty.sampleFrame = sampleImage;
	if (sampleImage.isAllocated())sampleFrames.push_back(currentProperty);
}

void videoContainer::startAnimation() {
	int count = sampleFrames.size()-1;
	animationStart = ofGetElapsedTimef();
	//int initialZValue = -100;
	//if(sampleFrames.size()>0)initialZValue = ((videoProperties&) sampleFrames.begin()).dimension.x;
	for (videoProperties& iteratorTemp : sampleFrames) {

		ofImage actualFrame = iteratorTemp.sampleFrame;
		int height = actualFrame.getHeight();
		int width = actualFrame.getWidth();
		int difference = (maxHeight - height) / 2;

		float drawX = displayCenter.x - width / 2;
		float drawY = displayCenter.y - height + (difference)+(count * 500);
		float drawZ = (float)(-300 - count * 1000);

		iteratorTemp.position = ofVec3f(drawX, drawY, drawZ);
		iteratorTemp.dimension = ofVec2f(width, height);
		count--;
	}
	readyState = true;

}

void videoContainer::reorderVideos(appUtils::VideoOrder videoOrder) {
	currentSorting = videoOrder;
	readyState = false;
	if (videoOrder == appUtils::VideoOrder::LengthAsc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecByLengthAsc);
	}
	else if (videoOrder == appUtils::VideoOrder::SizeAsc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecBySizeAsc);
	}
	else if (videoOrder == appUtils::VideoOrder::LengthDesc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecByLengthDesc);
	}
	else if (videoOrder == appUtils::VideoOrder::SizeDesc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecBySizeDesc);
	}
	startAnimation();
}