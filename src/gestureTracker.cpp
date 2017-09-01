#include "gestureTracker.h"

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

	frame frame; 
	frame.minX = 1;
	frame.maxX = depthPix.getWidth() - 1;

	frame.minY = depthPix.getHeight() * 0.25f;
	frame.maxY = depthPix.getHeight() * 0.75f;

	frame.width = frame.maxX - frame.minX;
	frame.height = frame.maxY - frame.minY;

	imageUtils::setFrame(frame, depthPix);
	
	imageUtils::setDepthCoordinatesSorted(depthCoords, frame);

	handImage = ofImage();
	imageUtils::setHandImage(handImage, frame);

	// calculate matching
	if (!featuresLoaded)return;

	std::array<float, 11 * 11> features{};
	imageUtils::setFeatureVector(handImage.getPixels(), features);
	int minDistance = numeric_limits<int>::max();
	for (std::array<float, 11 * 11> featuresRef : mouseFeaturesReference) {

		int difference = imageUtils::getEuclideanDist(featuresRef, features);
		minDistance = difference < minDistance ? difference : minDistance;
	}
	//cout << "Mouse Control:" << minDistance << endl;
	mouseAccuracy = minDistance;

	minDistance = numeric_limits<int>::max();
	for (std::array<float, 11 * 11> featuresRef : videoFeaturesReference) {

		float difference = imageUtils::getEuclideanDist(featuresRef, features);
		minDistance = difference < minDistance ? difference : minDistance;
	}
	//cout << "Video Control:" << minDistance << endl;
	videoAccuracy = minDistance;

	mouseAccuracy = (2000.f - mouseAccuracy) / 1500.f * 100.f;
	videoAccuracy = (2000.f - videoAccuracy) / 1500.f * 100.f;

	mouseAccuracy = mouseAccuracy < 0 ? 0 : (mouseAccuracy > 100.f ? 100.f : mouseAccuracy);
	videoAccuracy = videoAccuracy < 0 ? 0 : (videoAccuracy > 100.f ? 100.f : videoAccuracy);

	coordinateClusers.clear();
	if (mouseAccuracy > 50) {
		cursorMode = appUtils::CursorMode::Pointer;
		coordinateClusers.push_back(depthCoords.front());
	}
	else if (videoAccuracy > 50) {
		cursorMode = appUtils::CursorMode::Grab;
		int clusterRadius = 20;
		for (ofVec3f& iteratorTemp : depthCoords) {
			float x = iteratorTemp.x;
			float y = iteratorTemp.y;
			float z = iteratorTemp.z;
			if (coordinateClusers.size() == 5)break;
			if (z < (frame.minZ + 20)) {
				bool found = false;
				for (ofVec2f& iteratorCluster : coordinateClusers) {
					if (x >(iteratorCluster.x - clusterRadius)
						&& x <(iteratorCluster.x + clusterRadius)
						&& y >(iteratorCluster.y - clusterRadius)
						&& y < (iteratorCluster.y + clusterRadius)
						) {
						found = true;
					}
				}
				if (!found)coordinateClusers.push_back(ofVec2f(x, y));
			}
		}
	}
	else {
		cursorMode = appUtils::CursorMode::None;
	}
	delete[] frame.pixels;
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