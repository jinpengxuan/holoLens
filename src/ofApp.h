#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxDatGui.h"
#include "videoContainer.h"
#include "gestureTracker.h"
#include "appUtils.h"
#include "pathUtils.h"
#include "stringUtils.h"
#include "mouseCursor.h"
#include "menuWrapper.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void onButtonEvent(ofxDatGuiButtonEvent e);
	void onDropdownEvent(ofxDatGuiDropdownEvent e);
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofCamera cam;

	//time measurement
	float time;

	// video container for slides
	videoContainer videoContainer;

	// gesture Tracker class
	gestureTracker gestureTracker;

	//light
	ofLight light;

	vector<string> videoElements;

	//handCursor
	mouseCursor mouseCursor;

	const string capturePath = "c:\\of_v0.9.8_vs_release\\apps\\myApps\\holoLens\\bin\\data\\captures\\";

	//current parent folder
	string currentParent;

	menuWrapper menu;

private:

};
