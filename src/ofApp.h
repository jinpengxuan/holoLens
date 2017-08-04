#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void init();

	void testPCL();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	
	ofxKFW2::Device kinect;
	ofEasyCam cam;
	vector<ofVec2f> colorCoords;
	vector<ofVec3f> depthCoords;
	vector<ofVec3f> drawingCoords;

	//time measurement
	float time;

	ofImage handImage;

	//light
	ofLight light;

	ICoordinateMapper * coordinateMapper;
};
