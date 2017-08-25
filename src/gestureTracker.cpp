#include "gestureTracker.h"

bool sortVecByDepth(ofVec3f i, ofVec3f j) { return (i.z < j.z); }

void gestureTracker::init(vector<string> featureElements) {
	kinect.open();
	kinect.initDepthSource();
	//kinect.initColorSource();
	kinect.initInfraredSource();
	//kinect.initBodySource();
	//kinect.initBodyIndexSource();

	//colorCoords.resize(appUtils::DEPTH_SIZE);
	depthCoords.resize(appUtils::DEPTH_SIZE);
	//initFeatures(featureElements);
}

void gestureTracker::update() {
	kinect.update();

	HRESULT hresult = kinect.getSensor()->get_CoordinateMapper(&coordinateMapper);

	if (FAILED(hresult)) {
		ofLog() << "CoordinateMapper Not Found";
	}
	depthCoords.clear();

	const auto & depthPix = kinect.getDepthSource()->getPixels();

	//coordinateMapper->MapDepthFrameToColorSpace(appUtils::DEPTH_SIZE, (UINT16 *)depthPix.getPixels(), appUtils::DEPTH_SIZE, (ColorSpacePoint *)colorCoords.data());
	//cout <<  "Breite: " << depthPix.getWidth() << " -- Hoehe:" << depthPix.getHeight();

	/*kinect.getInfraredSource()->draw(0, 0, depthPix.getWidth(), depthPix.getHeight());*/
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
			if (distance < 400 || distance > 600) continue;
			//ofVec2f mappedCoord = colorCoords[index];

			//mappedCoord.x = floor(mappedCoord.x);
			//mappedCoord.y = floor(mappedCoord.y);

			// Make sure it's within some sane bounds, and skip it otherwise
			//if (mappedCoord.x < 0 || mappedCoord.y < 0 || mappedCoord.x >= DEPTH_WIDTH
			//	|| mappedCoord.y >= DEPTH_HEIGHT) {
			//	//cout << mappedCoord.x << " -- " << mappedCoord.y << "\n";
			//	continue;
			//}
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
				float xCoord = x;//appUtils::DEPTH_WIDTH - x + xShift;
				float yCoord = y;//appUtils::DEPTH_HEIGHT - y + yShift;
				float zCoord = distance;//distance + zShift;

				minZ = zCoord < minZ ? zCoord : minZ;
				minX = xCoord < minX ? xCoord : minX;
				maxX = xCoord > maxX ? xCoord : maxX;
				minY = yCoord < minY ? yCoord : minY;
				maxY = yCoord > maxY ? yCoord : maxY;

				//depthCoords.insert(depthCoords.begin(), ofVec3f(xCoord, yCoord, zCoord));
			}
		}
	}
	int widthImg = maxX - minX + 20;
	int heightImg = maxY - minY + 20;
	int maxZ = minZ + 50;

	handImage = ofImage();
	handImage.allocate(widthImg, heightImg, OF_IMAGE_GRAYSCALE);

	ofPixels pix = ofPixels();
	
	pix.allocate(widthImg, heightImg, OF_IMAGE_GRAYSCALE);
	pix.setColor(0);

	for (int y = minY; y < maxY; y += skip) {
		for (int x = minX; x < maxX; x += skip) {
			int index = x + y*depthPix.getWidth();
			int distance = depthPix[index];

			distance = distance > maxZ ? maxZ : distance;
			int thumbIndex = (x - minX + 10) + (y - minY + 10) * widthImg;
			float greyVal = (maxZ - distance) / 50.f * 255.f;
			pix.setColor(thumbIndex, ofColor(greyVal < 0 ? 0 : greyVal));
		}
	}

	handImage.setFromPixels(pix);
	handImage.resize(appUtils::HOG_SIZE, appUtils::HOG_SIZE);
	
	// calculate matching
	if (!featuresLoaded)return;

	std::array<float, 11 * 11> features{};
	appUtils::setFeatureVector(handImage.getPixels(), features);
	int minDistance = numeric_limits<int>::max();
	for (std::array<float, 11 * 11> featuresRef : mouseFeaturesReference) {

		int difference = appUtils::getEuclideanDist(featuresRef, features);
		minDistance = difference < minDistance ? difference : minDistance;
	}
	//cout << "Mouse Control:" << minDistance << endl;
	mouseAccuracy = minDistance;

	minDistance = numeric_limits<int>::max();
	for (std::array<float, 11 * 11> featuresRef : videoFeaturesReference) {

		float difference = appUtils::getEuclideanDist(featuresRef, features);
		minDistance = difference < minDistance ? difference : minDistance;
	}
	//cout << "Video Control:" << minDistance << endl;
	videoAccuracy = minDistance;
	
	/*
	sort(depthCoords.begin(), depthCoords.end(), sortVecByDepth);
	coordinateClusers.clear();

	int clusterRadius = 20;
	for (ofVec3f& iteratorTemp : depthCoords) {
		float x = iteratorTemp.x;
		float y = iteratorTemp.y;
		float z = iteratorTemp.z;
		if (z < (minZ + 10)) {
			bool found = false;
			for (ofVec3f& iteratorCluster : coordinateClusers) {
				if (x > (iteratorCluster.x - clusterRadius)
					&& x <(iteratorCluster.x + clusterRadius)
					&& y >(iteratorCluster.y - clusterRadius)
					&& y < (iteratorCluster.y + clusterRadius)
					) {
					found = true;
				}
			}
			if (!found)coordinateClusers.push_back(ofVec3f(x, y, z));
		}
	}

	//setze alle x Sekunden neue Kamera Position anhand der zentralen koordinate
	if ((int)(ofGetElapsedTimef() - time) % 10 == 0) {
		vector<ofVec3f>::iterator iteratorTemp;
		int sumX = 0;
		int sumY = 0;
		int count = 1;
		for (iteratorTemp = depthCoords.begin(); iteratorTemp < depthCoords.end(); iteratorTemp++) {
			sumX += ((ofVec3f)*iteratorTemp).x;
			sumY += ((ofVec3f)*iteratorTemp).y;
			count += 1;
		}
		sumX = sumX / count;
		sumY = sumY / count;

		center = ofVec3f(sumX, sumY, 5);
	}
	*/

	//--
	//Getting joint positions (skeleton tracking)
	//--
	//
	//{
	//	auto bodies = kinect.getBodySource()->getBodies();
	//	for (auto body : bodies) {
	//		for (auto joint : body.joints) {
	//			//now do something with the joints
	//		}
	//	}
	//}
	//
	//--



	//--
	//Getting bones (connected joints)
	//--
	//
	//{
	//	// Note that for this we need a reference of which joints are connected to each other.
	//	// We call this the 'boneAtlas', and you can ask for a reference to this atlas whenever you like
	//	auto bodies = kinect.getBodySource()->getBodies();
	//	auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

	//	for (auto body : bodies) {
	//		for (auto bone : boneAtlas) {
	//			auto firstJointInBone = body.joints[bone.first];
	//			auto secondJointInBone = body.joints[bone.second];

	//			//now do something with the joints
	//		}
	//	}
	//}
	//
	//--
}

void gestureTracker::draw() {

	if (handImage.isAllocated()) {
		int height = ofGetHeight();
		int width = ofGetWidth();
		handImage.resize(appUtils::HOG_SIZE, appUtils::HOG_SIZE);
		handImage.draw(0 - (width/3.5), 0 - (height/3.5));
	}

	/*check if hand position enables cursor functionality

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = appUtils::previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (appUtils::previewHeight - colorHeight) / 2.0;

	//kinect.getColorSource()->draw(0, 0 + colorTop, staticMembers.previewWidth, colorHeight);

	//kinect.getColorSource()->draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
	//kinect.getBodySource()->drawProjected(previewWidth, 0 + colorTop, previewWidth, colorHeight);

	ofSetColor(ofColor(255, 14, 120));
	for (ofVec3f& iteratorTemp : depthCoords) {
		if (iteratorTemp.z > (minZ + 80))continue;
		ofDrawCircle(ofPoint(iteratorTemp.x, iteratorTemp.y, iteratorTemp.z), 2);
	}
	ofSetColor(ofColor(255, 255, 255));

	//kinect.getInfraredSource()->draw(0, 0, previewWidth*2, previewHeight*2);
	//kinect.getDepthSource()->draw(0, 0, previewWidth, previewHeight);  // note that the depth texture is RAW so may appear dark

	//kinect.getBodyIndexSource()->draw(previewWidth, previewHeight, previewWidth, previewHeight);
	//kinect.getBodySource()->drawProjected(previewWidth, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);*/
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
	mouseAccuracy = numeric_limits<int>::max();
	videoAccuracy = numeric_limits<int>::max();
	// loop through elements and create features
	mouseFeaturesReference.clear();
	videoFeaturesReference.clear();

	vector<string>::iterator iteratorTemp;
	for (iteratorTemp = featureElements.begin(); iteratorTemp < featureElements.end(); iteratorTemp++) {
		string item = (string)*iteratorTemp;
		if (item.find("mouse") != std::string::npos && appUtils::hasEnding(item, (string)"png")) {
			ofImage testImage;
			testImage.load(item);
			std::array<float, 11 * 11> features{};
			appUtils::setFeatureVector(testImage.getPixels(), features);
			mouseFeaturesReference.push_back(features);
		}
		else if (item.find("video") != std::string::npos && appUtils::hasEnding(item, (string)"png")) {
			ofImage testImage;
			testImage.load(item);
			std::array<float, 11 * 11> features{};
			appUtils::setFeatureVector(testImage.getPixels(), features);
			videoFeaturesReference.push_back(features);
		}
	}
	featuresLoaded = true;
}

void gestureTracker::startDrag() {

}

void gestureTracker::stopDrag() {

}