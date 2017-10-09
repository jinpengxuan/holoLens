#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxDatGui.h"
#include "videoContainer.h"
#include "gestureTracker.h"
#include "applicationProperties.h"
#include "pathUtils.h"
#include "stringUtils.h"
#include "mouseCursor.h"
#include "menuWrapper.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

class controller : public ofBaseApp {

public:

	//lifecycle functions
	void setup();
	void update();
	void draw();

	//button events
	void onButtonEvent(ofxDatGuiButtonEvent e);
	void onDropdownEvent(ofxDatGuiDropdownEvent e);
	void keyPressed(int key);
	void keyReleased(int key);
	
	//mouse events
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	//camera object for the scene
	ofCamera cam;

	//time measurement
	float time;

	// video container for slides
	videoContainer videoContainer;

	// gesture Tracker class
	gestureTracker gestureTracker;

	//light
	ofLight light;

	//pathes of video elements currently loaded
	vector<string> videoElements;

	//handCursor
	mouseCursor mouseCursor;

	//capture path
	const string capturePath = "data\\captures\\";

	//current parent folder
	string currentParent;

	//menu wrapper object
	menuWrapper menu;

private:

};
