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

	depthCoords.clear();

	const auto & depthPix = kinect.getDepthSource()->getPixels();

	int skip = 1;
	minZ = numeric_limits<int>::max();
	minX = numeric_limits<int>::max();
	minY = numeric_limits<int>::max();
	maxX = numeric_limits<int>::min();
	maxY = numeric_limits<int>::min();

	if (depthPix.size() == 0)return;



	for (int y = 1; y < depthPix.getHeight() - 1; y += skip){
		for (int x = 1; x < depthPix.getWidth() - 1; x += skip) {
			int index = x + y*depthPix.getWidth();
			int distance = depthPix[index];
			if (distance < 400 || distance > 700) continue;
			
			// Outlier Removal

			int boundaryMax = distance + 10;
			int boundaryMin = distance - 10;
			int indexTop = index - depthPix.getWidth();
			int indexBottom = index + depthPix.getWidth();
			int indexLeft = index - 1;
			int indexRight = index + 1;
			if (depthPix[indexTop] < (boundaryMin) || depthPix[indexTop] > (boundaryMax) ||
				depthPix[indexBottom] < (boundaryMin) || depthPix[indexBottom] > (boundaryMax) ||
				depthPix[indexLeft] < (boundaryMin) || depthPix[indexLeft] > (boundaryMax) ||
				depthPix[indexRight] < (boundaryMin) || depthPix[indexRight] > (boundaryMax)) {
			}
			else {

				minZ = distance < minZ ? distance : minZ;
				
			}
		}
	}

	float difference = 80;
	float maxZ = minZ + difference;

	for (int y = 1; y < depthPix.getHeight() - 1; y += skip) {
		for (int x = 1; x < depthPix.getWidth() - 1; x += skip) {
			int index = x + y*depthPix.getWidth();
			int distance = depthPix[index];

			if (distance > maxZ || distance < minZ)continue;

			minX = x < minX ? x : minX;
			maxX = x > maxX ? x : maxX;
			minY = y < minY ? y : minY;
			maxY = y > maxY ? y : maxY;

			depthCoords.push_back(ofVec3f(x, y, distance));
		}
	}

	int widthImg = maxX - minX + 20;
	int heightImg = maxY - minY + 20;
	

	handImage = ofImage();
	handImage.allocate(widthImg, heightImg, OF_IMAGE_GRAYSCALE);

	ofPixels pix = ofPixels();
	
	pix.allocate(widthImg, heightImg, OF_IMAGE_GRAYSCALE);
	pix.setColor(0);

	for (int y = minY; y < maxY; y += skip) {
		for (int x = minX; x < maxX; x += skip) {
			int index = x + y*depthPix.getWidth();
			int distance = depthPix[index];

			int thumbIndex = (x - minX + 10) + (y - minY + 10) * widthImg;
			float greyVal = (maxZ - distance) / difference * 255.f;
			pix.setColor(thumbIndex, ofColor(greyVal < 0 ? 0 : greyVal));
		}
	}

	handImage.setFromPixels(pix);
	handImage.resize(appUtils::HOG_SIZE, appUtils::HOG_SIZE);
	
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
		sort(depthCoords.begin(), depthCoords.end(), sortVecByDepth);
		cursorMode = appUtils::CursorMode::Pointer;
		coordinateClusers.push_back(depthCoords.front());
	}
	else if (videoAccuracy > 50) {
		sort(depthCoords.begin(), depthCoords.end(), sortVecByDepth);
		cursorMode = appUtils::CursorMode::Grab;
		int clusterRadius = 20;
		for (ofVec3f& iteratorTemp : depthCoords) {
			float x = iteratorTemp.x;
			float y = iteratorTemp.y;
			float z = iteratorTemp.z;
			if (coordinateClusers.size() == 5)break;
			if (z < (minZ + 20)) {
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
}

void gestureTracker::draw() {
	
	if (handImage.isAllocated()) {
		int height = ofGetHeight();
		int width = ofGetWidth();
		handImage.resize(appUtils::HOG_SIZE, appUtils::HOG_SIZE);
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