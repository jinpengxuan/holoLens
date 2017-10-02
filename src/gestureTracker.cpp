#include "gestureTracker.h"

bool sortVecByDepth(ofVec3f i, ofVec3f j) { return (i.z < j.z); }

void gestureTracker::init(vector<string> featureElements) {
	kinect.open();
	kinect.initDepthSource();
	kinect.initInfraredSource();
	kinect.initColorSource();
	//depthCoords.resize(appUtils::DEPTH_SIZE);
	initFeatures(featureElements);
}

void gestureTracker::update() {
	kinect.update();
	if (!kinect.isFrameNew())return;

	const auto & depthPix = kinect.getDepthSource()->getPixels();

	kinect.getColorSource()->setRgbaPixelsEnabled(true);
	ofPixels & colorPix = kinect.getColorSource()->getPixels();

	int skip = 1;

	if (depthPix.size() == 0 || colorPix.size() == 0)return;

	// set frame positions for evaluation
	frame frame; 
	frame.minX = 50;
	frame.maxX = depthPix.getWidth() - 50;

	frame.minY = depthPix.getHeight() * 0.20f;
	frame.maxY = depthPix.getHeight() * 0.80f;

	frame.width = frame.maxX - frame.minX;
	frame.height = frame.maxY - frame.minY;

	// get a slightly smaller frame from the depth frame
	imageUtils::setFrame(frame, depthPix);
	
	// set depth coordinates and frame to evaluate
	imageUtils::setDepthCoordinates(frame);

	if (ofGetElapsedTimeMillis() - checkGestureTime >= 500 && (frame.widthImg > 20 && frame.heightImg > 20)) {
		checkGestureTime = ofGetElapsedTimeMillis();
		// set the image of the depth coordinates from the nearest object
		handDepthImage = ofImage();
		handColorImage = ofImage();
		imageUtils::setHandDepthImage(handDepthImage, frame);
		imageUtils::setHandColorImage(handColorImage, frame, depthPix, colorPix);

		// calculate matching
		if (!featuresLoaded)return;

		// get features of current frame
		std::array<float, 11 * 11> features{};
		imageUtils::setFeatureVector(handDepthImage.getPixels(), features);

		// get accuracies
		mouseAccuracy = imageUtils::getAccuracy(mouseFeaturesReference, features);
		videoAccuracy = imageUtils::getAccuracy(videoFeaturesReference, features);
		abortAccuracy = imageUtils::getAccuracy(abortFeaturesReference, features);

		if (mouseAccuracy > 60) {
			cursorMode = applicationProperties::CursorMode::Pointer;
		}
		else if (videoAccuracy > 50) {
			cursorMode = applicationProperties::CursorMode::Grab;
		}
		else if (abortAccuracy > 60) {
			cursorMode = applicationProperties::CursorMode::None;
		}
	}

	// get clusters of finger tips
	coordinateClusers.clear();
	if (cursorMode == applicationProperties::CursorMode::Pointer) {
		coordinateClusers.push_back(frame.nearPoint);
	}
	else if (cursorMode == applicationProperties::CursorMode::Grab) {
		imageUtils::setPixelClusters(coordinateClusers, frame);
	}
	else {
		cursorMode = applicationProperties::CursorMode::None;
	}
	delete[] frame.pixels;
}

void gestureTracker::draw() {
	
	if (handDepthImage.isAllocated()) {
		int height = ofGetHeight();
		int width = ofGetWidth();
		handDepthImage.draw(0 - (width/3.5), 0 - (height/3.5));
	}
}

void gestureTracker::capture(string gestureType) {
	if (handDepthImage.isAllocated()) {
		if (handDepthImage.isAllocated()) {
			handDepthImage.resize(applicationProperties::HOG_SIZE, applicationProperties::HOG_SIZE);
			handDepthImage.save("captures\\" + gestureType + to_string(rand() % 1000000) + ".png", ofImageQualityType::OF_IMAGE_QUALITY_HIGH);
		}
		if (handColorImage.isAllocated()) {
			handColorImage.save("captures\\" + gestureType + to_string(rand() % 1000000) + "-color.png", ofImageQualityType::OF_IMAGE_QUALITY_HIGH);
		}
	}
}

void gestureTracker::initFeatures(vector<string> features) {
	featuresLoaded = false;
	mouseAccuracy = 0;
	videoAccuracy = 0;
	mouseFeaturesReference.clear();
	videoFeaturesReference.clear();
	abortFeaturesReference.clear();

	// loop through elements and create features
	for (vector<string>::iterator i = features.begin(); i < features.end(); i++) {

		string item = (string)*i;

		if (!stringUtils::hasEnding(item, (string)"png"))
		{
			continue;
		}

		if (stringUtils::contains(item, "mouse")) {
			addFeature(mouseFeaturesReference, item);
			continue;
		}

		if (stringUtils::contains(item, "video")) {
			addFeature(videoFeaturesReference, item);
			continue;
		}

		if (stringUtils::contains(item, "abort")) {
			addFeature(abortFeaturesReference, item);
		}
	}

	featuresLoaded = mouseFeaturesReference.size() != 0 || videoFeaturesReference.size() != 0;

}

void gestureTracker::addFeature(vector<std::array<float, 11 * 11>>& featuresReference, string item) {
	ofImage testImage;
	testImage.load(item);
	std::array<float, 11 * 11> features{};
	imageUtils::setFeatureVector(testImage.getPixels(), features);
	featuresReference.push_back(features);
}

void gestureTracker::startDrag() {

}

void gestureTracker::stopDrag() {

}