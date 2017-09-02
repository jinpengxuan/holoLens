#include "gestureTracker.h"

bool sortVecByDepth(ofVec3f i, ofVec3f j) { return (i.z < j.z); }

void gestureTracker::init(vector<string> featureElements) {
	kinect.open();
	kinect.initDepthSource();
	kinect.initInfraredSource();
	depthCoords.resize(appUtils::DEPTH_SIZE);
}

void gestureTracker::update() {
	kinect.update();

	const auto & depthPix = kinect.getDepthSource()->getPixels();

	int skip = 1;

	if (depthPix.size() == 0)return;

	// set frame positions for evaluation
	frame frame; 
	frame.minX = 1;
	frame.maxX = depthPix.getWidth() - 1;

	frame.minY = depthPix.getHeight() * 0.15f;
	frame.maxY = depthPix.getHeight() * 0.85f;

	frame.width = frame.maxX - frame.minX;
	frame.height = frame.maxY - frame.minY;

	// get a slightly smaller frame from the depth frame
	imageUtils::setFrame(frame, depthPix);
	
	// set depth coordinates and frame to evaluate
	imageUtils::setDepthCoordinates(depthCoords, frame);

	// set the image of the depth coordinates from the nearest object
	handImage = ofImage();
	imageUtils::setHandImage(handImage, frame);

	delete[] frame.pixels;
	// calculate matching
	if (!featuresLoaded)return;

	// get features of current frame
	std::array<float, 11 * 11> features{};
	imageUtils::setFeatureVector(handImage.getPixels(), features);

	// get accuracies
	mouseAccuracy = imageUtils::getAccuracy(mouseFeaturesReference, features);
	videoAccuracy = imageUtils::getAccuracy(videoFeaturesReference, features);

	// get clusters of finger tips
	coordinateClusers.clear();
	if (mouseAccuracy > 50) {
		sort(depthCoords.begin(), depthCoords.end(), sortVecByDepth);
		cursorMode = appUtils::CursorMode::Pointer;
		imageUtils::setClusters(depthCoords, coordinateClusers, frame, 1);
	}
	else if (videoAccuracy > 50) {
		sort(depthCoords.begin(), depthCoords.end(), sortVecByDepth);
		cursorMode = appUtils::CursorMode::Grab;
		imageUtils::setClusters(depthCoords, coordinateClusers, frame, 5);
	}
	else {
		cursorMode = appUtils::CursorMode::None;
	}
}

void gestureTracker::draw() {
	
	if (handImage.isAllocated()) {
		int height = ofGetHeight();
		int width = ofGetWidth();
		handImage.draw(0 - (width/3.5), 0 - (height/3.5));
	}
}

void gestureTracker::capture(string gestureType) {
	if (handImage.isAllocated()) {
		if (handImage.isAllocated()) {
			std::array<float, 11 * 11> features;
			handImage.resize(appUtils::HOG_SIZE, appUtils::HOG_SIZE);
			handImage.save("captures\\" + gestureType + to_string(rand() % 1000000) + ".png", ofImageQualityType::OF_IMAGE_QUALITY_HIGH);
		}
	}
}

void gestureTracker::initFeatures(vector<string> featureElements) {
	featuresLoaded = false;
	mouseAccuracy = 0;
	videoAccuracy = 0;
	// loop through elements and create features
	mouseFeaturesReference.clear();
	videoFeaturesReference.clear();

	vector<string>::iterator iteratorTemp;
	for (iteratorTemp = featureElements.begin(); iteratorTemp < featureElements.end(); iteratorTemp++) {
		string item = (string)*iteratorTemp;
		if (item.find("mouse") != std::string::npos && stringUtils::hasEnding(item, (string)"png")) {
			ofImage testImage;
			testImage.load(item);
			std::array<float, 11 * 11> features{};
			imageUtils::setFeatureVector(testImage.getPixels(), features);
			mouseFeaturesReference.push_back(features);
		}
		else if (item.find("video") != std::string::npos && stringUtils::hasEnding(item, (string)"png")) {
			ofImage testImage;
			testImage.load(item);
			std::array<float, 11 * 11> features{};
			imageUtils::setFeatureVector(testImage.getPixels(), features);
			videoFeaturesReference.push_back(features);
		}
	}
	if(!mouseFeaturesReference.size()==0 || !videoFeaturesReference.size() == 0)featuresLoaded = true;
}

void gestureTracker::startDrag() {

}

void gestureTracker::stopDrag() {

}