#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxDatGui.h"
#include <iostream>
#include "videoContainer.h"
#include "gestureTracker.h"
#include "utils.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void testPCL();

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

	ofEasyCam cam;

	//time measurement
	float time;

	// video container for slides
	videoContainer videoContainer;

	// gesture Tracker class
	gestureTracker gestureTracker;

	//light
	ofLight light;

	vector<string> videoElements;

	//UI Objects -----------------------------------
	int elements = 0;
	bool isReady = false;

	std::array<string, 6> drives{ { "c:\\","d:\\","e:\\","f:\\","g:\\","h:\\" } };

	//gui panel objects
	ofxDatGui* fileSystemGui;
	ofxDatGui* framerateGui;

	//gui buttons
	ofxDatGuiButton* openButton;
	ofxDatGuiButton* upButton;

	ofxDatGuiLabel* pathLabel;

	string actualPath;

	//available drives list
	vector <string> availableDrives;

	//current parent folder
	string currentParent;

private:

	void loadSubOptions(string directory);
	void setVideoElements(string path);

};
