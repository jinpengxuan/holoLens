#include "gestureTracker.h"

void gestureTracker::init() {
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	colorCoords.resize(staticMembers.DEPTH_SIZE);
	depthCoords.resize(staticMembers.DEPTH_SIZE);

	//handImage = ofImage();
	//handImage.allocate(staticMembers.DEPTH_WIDTH, staticMembers.DEPTH_HEIGHT, OF_IMAGE_COLOR);
}

void gestureTracker::update() {
	kinect.update();

	HRESULT hresult = kinect.getSensor()->get_CoordinateMapper(&coordinateMapper);

	if (FAILED(hresult)) {
		ofLog() << "CoordinateMapper Not Found";
	}
	depthCoords.clear();
	std::vector<ofVec3f>::iterator iteratorDepth = depthCoords.begin();

	const auto & depthPix = kinect.getDepthSource()->getPixels();

	coordinateMapper->MapDepthFrameToColorSpace(staticMembers.DEPTH_SIZE, (UINT16 *)depthPix.getPixels(), staticMembers.DEPTH_SIZE,
		(ColorSpacePoint *)colorCoords.data());
	//cout <<  "Breite: " << depthPix.getWidth() << " -- Hoehe:" << depthPix.getHeight();

	/*kinect.getInfraredSource()->draw(0, 0, depthPix.getWidth(), depthPix.getHeight());*/
	int skip = 5;
	minZ = std::numeric_limits<int>::max();

	if (depthPix.size() == 0)return;
	for (int x = 1; x < depthPix.getWidth() - 1; x += skip) {
		for (int y = 1; y < depthPix.getHeight() - 1; y += skip) {
			int index = x + y*depthPix.getWidth();
			int distance = depthPix[index];
			//ofVec2f mappedCoord = colorCoords[index];

			//mappedCoord.x = floor(mappedCoord.x);
			//mappedCoord.y = floor(mappedCoord.y);

			// Make sure it's within some sane bounds, and skip it otherwise
			//if (mappedCoord.x < 0 || mappedCoord.y < 0 || mappedCoord.x >= DEPTH_WIDTH
			//	|| mappedCoord.y >= DEPTH_HEIGHT) {
			//	//cout << mappedCoord.x << " -- " << mappedCoord.y << "\n";
			//	continue;
			//}

			if (distance > 350 && distance < 800) {

				// Outlier Removal
				int boundaryMax = distance + 10;
				int boundaryMin = distance - 10;
				int indexTop = index - depthPix.getWidth();
				int indexBottom = index + depthPix.getWidth();
				int indexLeft = index + depthPix.getWidth();
				int indexRight = index + depthPix.getWidth();
				if (depthPix[indexTop] < (boundaryMin) || depthPix[indexTop] > (boundaryMax) ||
					depthPix[indexBottom] < (boundaryMin) || depthPix[indexBottom] > (boundaryMax) ||
					depthPix[indexLeft] < (boundaryMin) || depthPix[indexLeft] > (boundaryMax) ||
					depthPix[indexRight] < (boundaryMin) || depthPix[indexRight] > (boundaryMax)) {
				}
				else {
					if (distance < minZ) {
						minZ = distance;
					}
					depthCoords.insert(iteratorDepth, ofVec3f(x, y, distance));
				}
			}
			else {
				//ofSetColor(ofColor(0));
				//ofDrawLine(ofPoint(x - (depthPix.getWidth() / 2), y, 0), ofPoint(x, y, distance));
			}
		}
	}

	//setze alle x Sekunden neue Kamera Position anhand der zentralen koordinate
	if ((int)(ofGetElapsedTimef() - time) % 10 == 0) {
		std::vector<ofVec3f>::iterator iteratorTemp;
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
	std::vector<ofVec3f>::iterator iteratorTemp;

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = staticMembers.previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (staticMembers.previewHeight - colorHeight) / 2.0;

	//kinect.getColorSource()->draw(0, 0 + colorTop, staticMembers.previewWidth, colorHeight);

	//kinect.getColorSource()->draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
	//kinect.getBodySource()->drawProjected(previewWidth, 0 + colorTop, previewWidth, colorHeight);

	ofSetColor(ofColor(255, 14, 120));
	for (iteratorTemp = depthCoords.begin(); iteratorTemp < depthCoords.end(); iteratorTemp++) {
		if (((ofVec3f)*iteratorTemp).z < (minZ + 50)) {
			ofSetColor(ofColor(255, 14, 120));
			ofDrawSphere(ofPoint(staticMembers.DEPTH_WIDTH - ((ofVec3f)*iteratorTemp).x + xShift, staticMembers.DEPTH_HEIGHT - ((ofVec3f)*iteratorTemp).y + yShift, ((ofVec3f)*iteratorTemp).z + zShift) , 2);
		}
		else {
			ofSetColor(ofColor(255, 255, 255));
			ofDrawSphere(ofPoint(staticMembers.DEPTH_WIDTH - ((ofVec3f)*iteratorTemp).x + xShift, staticMembers.DEPTH_HEIGHT - ((ofVec3f)*iteratorTemp).y + yShift, ((ofVec3f)*iteratorTemp).z + zShift), 2);
		}
	}
	ofSetColor(ofColor(255, 255, 255));

	//kinect.getInfraredSource()->draw(0, 0, previewWidth*2, previewHeight*2);
	//kinect.getDepthSource()->draw(0, 0, previewWidth, previewHeight);  // note that the depth texture is RAW so may appear dark

	//kinect.getBodyIndexSource()->draw(previewWidth, previewHeight, previewWidth, previewHeight);
	//kinect.getBodySource()->drawProjected(previewWidth, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);
}

void gestureTracker::startDrag() {

}

void gestureTracker::stopDrag() {

}