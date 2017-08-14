#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include <iostream>
#include "videoContainer.h"
#include "gestureTracker.h"
#include "menu.h"
#include "staticMembers.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

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

	ofEasyCam cam;

	//time measurement
	float time;

	// video container for slides
	videoContainer videoContainer;

	// gesture Tracker class
	gestureTracker gestureTracker;

	// menu wrapper
	menu menu;

	//light
	ofLight light;

	staticMembers staticMembers;
};
