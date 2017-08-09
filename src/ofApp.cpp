#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

	//ofSetWindowShape(previewWidth * 2, previewHeight * 2);
	ofSetWindowShape(staticMembers.WINDOW_WIDTH, staticMembers.WINDOW_HEIGHT);

	cam = ofEasyCam();
	cam.setPosition(ofVec3f(0, 0, 5));
	cam.lookAt(ofVec3f(0, 0, 0));

	ofSetBackgroundColor(ofColor(0));

	//get the starting time
	time = ofGetElapsedTimef();

	// set colors an positions for the lights and the material
	light.setDiffuseColor(ofColor(0.f, 255.f, 0.f));
	light.setAmbientColor(ofColor(255.f, 255.f, 255.f));
	light.setPosition(0, 0, -2000);

	//Video Init
	videoContainer.init();

	//Gesture Init
	gestureTracker.init();

	//ofVec3f pos = ofVec3f(0,0,-1);
	//ofVec3f target = ofVec3f(0, 0, 0);

	//cam = ofCamera();
	//cam.lookAt(target);
	//cam.setPosition(pos);
}

//--------------------------------------------------------------
void ofApp::update() {
	gestureTracker.update();
}

//--------------------------------------------------------------
void ofApp::draw() {

	cam.begin();

	videoContainer.draw(ofVec2f(0,0));

	gestureTracker.draw();

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
