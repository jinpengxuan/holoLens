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
	sort(sampleFrames.begin(), sampleFrames.end(), sortVecBySizeDesc);


	displayCenter = center;

	startAnimation();
}

void videoContainer::update() {
	if (!readyState)return;
	if (!actualVideo.isPaused()) actualVideo.update();

	alphaValue = 255;
	zAnimation = 0;
	if ((ofGetElapsedTimef() - animationStart) <= animationTime) {
		float timeParameter = (ofGetElapsedTimef() - animationStart) / animationTime;
		alphaValue = timeParameter * 255;
		zAnimation = (1 - timeParameter) * 50;
	}
}

void videoContainer::draw() {
	if (!readyState)return;

	ofEnableAlphaBlending();
	ofSetColor(255, 255, 255, alphaValue);
	int count = sampleFrames.size()-1;
	for (videoProperties& iteratorTemp : sampleFrames) {

		if (count > 0) {
			ofImage actualFrame = iteratorTemp.sampleFrame;
			actualFrame.allocate(iteratorTemp.dimension.x, iteratorTemp.dimension.y, OF_IMAGE_COLOR);
			actualFrame.draw(iteratorTemp.position.x, iteratorTemp.position.y, iteratorTemp.position.z - zAnimation, iteratorTemp.dimension.x, iteratorTemp.dimension.y);
		}
		else {
			actualVideo.draw(ofPoint(iteratorTemp.position.x, iteratorTemp.position.y, iteratorTemp.position.z - zAnimation), iteratorTemp.dimension.x, iteratorTemp.dimension.y);
		}
		count--;
	}
	ofDisableAlphaBlending();
}

void videoContainer::pause(bool paused) {
	actualVideo.setPaused(paused);
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
	sampleImage.getHeight();

	currentProperty.sampleFrame = sampleImage;
	currentProperty.name = path;// .substr(path.find_last_of("\\") + 1);
	if (sampleImage.isAllocated())sampleFrames.push_back(currentProperty);
}

void videoContainer::startAnimation() {
	int count = sampleFrames.size()-1;
	animationStart = ofGetElapsedTimef();
	//int initialZValue = -100;
	//if(sampleFrames.size()>0)initialZValue = ((videoProperties&) sampleFrames.begin()).dimension.x;
	for (videoProperties& iteratorTemp : sampleFrames) {

		ofImage actualFrame = iteratorTemp.sampleFrame;
		int height = actualFrame.getHeight() > maxHeight ? maxHeight : actualFrame.getHeight();
		int width = actualFrame.getHeight() > maxHeight ? (int)(maxHeight / (float)actualFrame.getHeight() * actualFrame.getWidth()) : actualFrame.getWidth();

		float drawX = displayCenter.x - width / 2;
		float drawY = displayCenter.y - height + 100 + (count * 300);
		float drawZ = (float)(-300 - count * 500);

		iteratorTemp.position = ofVec3f(drawX, drawY, drawZ);
		iteratorTemp.dimension = ofVec2f(width, height);
		iteratorTemp.sampleFrame.resize(width, height);
		count--;
	}

	actualVideo.closeMovie();
	actualVideo.load(sampleFrames.back().name);
	actualVideo.setVolume(0);
	actualVideo.play();
	float pct = 0.01f;
	actualVideo.setPosition(pct);
	actualVideo.update();
	actualVideo.setPaused(true);
	videoName = sampleFrames.front().name;

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