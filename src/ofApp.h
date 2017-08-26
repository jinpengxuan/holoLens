#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxDatGui.h"
#include "videoContainer.h"
#include "gestureTracker.h"
#include "appUtils.h"
#include "mouseCursor.h"

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

	const enum MediaType { Video, Feature };

	ofCamera cam;

	bool collapsed = false;

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

	//UI Objects ----------------------------------------------------------------------
	int elements = 0;

	array<string, 6> drives{ { "c:\\","d:\\","e:\\","f:\\","g:\\","h:\\" } };

	//gui panel objects
	ofxDatGui* fileSystemGui;
	ofxDatGui* framerateGui;
	ofxDatGui* sortingGui;

	//main menu options
	ofxDatGuiLabel* videoNameLabel;
	ofxDatGuiButton* collapseButton;
	ofxDatGuiButton* playButton;
	ofxDatGuiButton* evaluateButton;
	ofxDatGuiButton* learnMouseControlButton;
	ofxDatGuiButton* learnVideoControlButton;
	ofxDatGuiSlider* mouseControlPrecision;
	ofxDatGuiSlider* videoControlPrecision;

	//gui filesystem elements
	ofxDatGuiButton* openButton;
	ofxDatGuiButton* upButton;

	ofxDatGuiLabel* pathLabel;

	const string capturePath = "c:\\of_v0.9.8_vs_release\\apps\\myApps\\holoLens\\bin\\data\\captures\\";

	//gui sorting elements
	ofxDatGuiDropdown* sortOptions;

	//available drives list
	vector <string> availableDrives;

	//current parent folder
	string currentParent;

private:

	void loadSubOptions(string directory);
	void setPathElements(vector<string>& elements, string path, MediaType type);

};
