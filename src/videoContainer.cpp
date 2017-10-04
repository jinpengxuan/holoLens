#include "videoContainer.h"

bool sortVecByLengthAsc(videoProperties i, videoProperties j) { return (i.duration < j.duration); }
bool sortVecBySizeAsc(videoProperties i, videoProperties j) { return ((i.dimension.x + i.dimension.y) < (j.dimension.x + j.dimension.y)); }

bool sortVecByLengthDesc(videoProperties i, videoProperties j) { return (i.duration > j.duration); }
bool sortVecBySizeDesc(videoProperties i, videoProperties j) { return ((i.dimension.x + i.dimension.y) > (j.dimension.x + j.dimension.y)); }

void videoContainer::init(ofVec2f center, vector<string> elements) {
	// loop through directory and create video visual
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
	if (!actualVideo.isPaused() && actualVideo.getSpeed() == 1.f) {
		actualVideo.update();
	}
	/*else if (!actualVideo.isPaused() && actualVideo.getSpeed() < 0 && ofGetElapsedTimef() - videoControlTime >= 100.f) {
		for (int i = 0; i < abs(actualVideo.getSpeed()) + 2; i++) {
			actualVideo.previousFrame();
		}
		actualVideo.update();
		videoControlTime = ofGetElapsedTimef();
	}
	else if (!actualVideo.isPaused() && ofGetElapsedTimef() - videoControlTime >= 100.f) {
		int frameNumber = actualVideo.getCurrentFrame() + actualVideo.getSpeed();
		frameNumber = frameNumber < actualVideo.getTotalNumFrames() ? frameNumber : actualVideo.getTotalNumFrames() - 1;
		actualVideo.setFrame(frameNumber);
		actualVideo.update();
		videoControlTime = ofGetElapsedTimef();
	}*/

	initAlphaValue = 255;
	zAnimation = 0;
	if ((ofGetElapsedTimef() - initAnimationStart) <= initAnimationTime) {
		float timeParameter = (ofGetElapsedTimef() - initAnimationStart) / initAnimationTime;
		initAlphaValue = timeParameter * 255;
		zAnimation = (1 - timeParameter) * 100;
	} 
	else if ((ofGetElapsedTimef() - dismissAnimationStart) <= dismissAnimationTime) {
		float timeParameter = (ofGetElapsedTimef() - dismissAnimationStart) / dismissAnimationTime;
		dismissAlphaValue = (1 - timeParameter) * 255;
		xAnimation = timeParameter * 300;
	}
	else if (dismissing) {
		dismissAlphaValue = 255;
		xAnimation = 0;
		dismissing = false;
		if(sampleFrames.size()>0)sampleFrames.erase(sampleFrames.end()-1);
		setVisualProperties();
	}
}

void videoContainer::draw() {

	ofEnableAlphaBlending();
	ofSetColor(255, 255, 255, initAlphaValue);
	int count = 0;
	for (videoProperties& iteratorTemp : sampleFrames) {

		if (count < sampleFrames.size() - 1) {
			float xShiftSample = count % 2 == 0 ? -300 : 300;
			ofImage actualFrame = iteratorTemp.sampleFrame;
			actualFrame.allocate(iteratorTemp.dimension.x, iteratorTemp.dimension.y, OF_IMAGE_COLOR);
			actualFrame.draw(iteratorTemp.position.x + xShiftSample, iteratorTemp.position.y, iteratorTemp.position.z - zAnimation, iteratorTemp.dimension.x, iteratorTemp.dimension.y);
		}
		else {
			if(dismissAlphaValue<255)ofSetColor(255, 255, 255, dismissAlphaValue);
			float scalingXShift = (scaling - 1.f) * iteratorTemp.dimension.x / 2.f;
			float scalingYShift = (scaling - 1.f) * iteratorTemp.dimension.y / 2.f;
			actualVideo.draw(ofPoint(iteratorTemp.position.x + xAnimation - scalingXShift, iteratorTemp.position.y - scalingYShift, iteratorTemp.position.z - zAnimation), iteratorTemp.dimension.x * scaling, iteratorTemp.dimension.y * scaling);
		}
		count++;
	}
	ofDisableAlphaBlending();
}

void videoContainer::pause(bool paused) {
	isPaused = paused;
	actualVideo.setPaused(paused);
}

void videoContainer::setSpeed(float speed) {
	actualVideo.setSpeed(speed);
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
	initAnimationStart = ofGetElapsedTimef();
	setVisualProperties();

}

void videoContainer::setVisualProperties() {
	if (sampleFrames.size() == 0)return;
	int count = sampleFrames.size() - 1;
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
	actualVideo.setPaused(isPaused);
	videoName = sampleFrames.back().name;
}

void videoContainer::dismissVideo() {
	dismissAnimationStart = ofGetElapsedTimef();
	dismissing = true;
}

void videoContainer::reorderVideos(applicationProperties::VideoOrder videoOrder) {
	currentSorting = videoOrder;
	if (videoOrder == applicationProperties::VideoOrder::LengthAsc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecByLengthAsc);
	}
	else if (videoOrder == applicationProperties::VideoOrder::SizeAsc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecBySizeAsc);
	}
	else if (videoOrder == applicationProperties::VideoOrder::LengthDesc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecByLengthDesc);
	}
	else if (videoOrder == applicationProperties::VideoOrder::SizeDesc) {
		sort(sampleFrames.begin(), sampleFrames.end(), sortVecBySizeDesc);
	}
	startAnimation();
}