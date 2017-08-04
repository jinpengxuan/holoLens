#include "ofApp.h"

int previewWidth = 640;
int previewHeight = 480;
int DEPTH_WIDTH = 512;
int DEPTH_HEIGHT = 424;
int DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

void ofApp::init() {

}

//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	//ofSetWindowShape(previewWidth * 2, previewHeight * 2);
	ofSetWindowShape(DEPTH_WIDTH, DEPTH_HEIGHT);

	cam = ofEasyCam();
	cam.setPosition(ofVec3f(0, 0, 5));
	cam.lookAt(ofVec3f(0, 0, 0));

	ofSetBackgroundColor(ofColor(0));

	colorCoords.resize(DEPTH_SIZE);
	depthCoords.resize(DEPTH_SIZE);

	handImage = ofImage();
	handImage.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);

	//get the starting time
	time = ofGetElapsedTimef();

	// set colors an positions for the lights and the material
	light.setDiffuseColor(ofColor(0.f, 255.f, 0.f));
	light.setAmbientColor(ofColor(255.f, 255.f, 255.f));
	light.setPosition(0, 0, -2000);

	//Video Init
	videoContainer.init();

	//ofVec3f pos = ofVec3f(0,0,-1);
	//ofVec3f target = ofVec3f(0, 0, 0);

	//cam = ofCamera();
	//cam.lookAt(target);
	//cam.setPosition(pos);
}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	HRESULT hresult = kinect.getSensor()->get_CoordinateMapper(&coordinateMapper);

	if (FAILED(hresult)) {
		ofLog() << "CoordinateMapper Not Found";
	}
	depthCoords.clear();
	std::vector<ofVec3f>::iterator iteratorDepth = depthCoords.begin();

	const auto & depthPix = kinect.getDepthSource()->getPixels();

	coordinateMapper->MapDepthFrameToColorSpace(DEPTH_SIZE, (UINT16 *)depthPix.getPixels(), DEPTH_SIZE,
		(ColorSpacePoint *)colorCoords.data());
	//cout <<  "Breite: " << depthPix.getWidth() << " -- Hoehe:" << depthPix.getHeight();

	/*kinect.getInfraredSource()->draw(0, 0, depthPix.getWidth(), depthPix.getHeight());*/
	int skip = 5;

	if (depthPix.size() == 0)return;
	for (int x = 1; x < depthPix.getWidth()-1; x += skip) {
		for (int y = 1; y < depthPix.getHeight()-1; y += skip) {
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

			if (distance > 350 && distance < 800){

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
		int count = 0;
		for (iteratorTemp = depthCoords.begin(); iteratorTemp < depthCoords.end(); iteratorTemp++) {
			sumX += ((ofVec3f)*iteratorTemp).x;
			sumY += ((ofVec3f)*iteratorTemp).y;
			count += 1;
		}
		sumX = sumX / count;
		sumY = sumY / count;

		cam.setPosition(ofVec3f(sumX, sumY, 5));
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

//--------------------------------------------------------------
void ofApp::draw() {

	std::vector<ofVec3f>::iterator iteratorTemp;
	cam.begin();

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (previewHeight - colorHeight) / 2.0;

	kinect.getColorSource()->draw(0, 0 + colorTop, previewWidth, colorHeight);

	//kinect.getColorSource()->draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
	//kinect.getBodySource()->drawProjected(previewWidth, 0 + colorTop, previewWidth, colorHeight);

	for (iteratorTemp = depthCoords.begin(); iteratorTemp < depthCoords.end(); iteratorTemp++) {
		ofSetColor(ofColor(255, 14, 120));
		ofDrawSphere(ofPoint(DEPTH_WIDTH - ((ofVec3f)*iteratorTemp).x, DEPTH_HEIGHT - ((ofVec3f)*iteratorTemp).y, -((ofVec3f)*iteratorTemp).z), 2);
	}

	videoContainer.draw(ofVec2f(0,0));

	//kinect.getInfraredSource()->draw(0, 0, previewWidth*2, previewHeight*2);
	//kinect.getDepthSource()->draw(0, 0, previewWidth, previewHeight);  // note that the depth texture is RAW so may appear dark

	//kinect.getBodyIndexSource()->draw(previewWidth, previewHeight, previewWidth, previewHeight);
	//kinect.getBodySource()->drawProjected(previewWidth, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);

	cam.end();
}

void ofApp::testPCL() {
	/*pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
	cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
	cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
	cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

	for (size_t i = 0; i < cloud.points.size(); ++i)
	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;*/
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
