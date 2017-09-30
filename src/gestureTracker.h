#pragma once

#include "ofxKinectForWindows2.h"
#include "applicationProperties.h"
#include "frame.h"
#include "imageUtils.h"
#include "stringUtils.h"
#include <limits>

class gestureTracker {

public:
	
	void startDrag();
	void stopDrag();
	void init(vector<string> featureElements);
	void update();
	void draw();
	void capture(string gestureType);
	void initFeatures(vector<string> featureElements);

	ofxKFW2::Device kinect;

	bool dragged = false;
	bool featuresLoaded = false;

	applicationProperties::CursorMode cursorMode = applicationProperties::CursorMode::None;
	vector<ofVec2f> cursorPositions;

	int trackingTime = 0;

	float rotationDegree = 0.f;
	int translation = 0;

	int xShift = -200;
	int yShift = -400;
	int zShift = -300;

	//vector<ofVec2f> colorCoords;
	//vector<ofVec3f> depthCoords;
	//vector<ofVec3f> drawingCoords;

	vector<ofVec3f> coordinateClusers;

	//central position
	ofVec3f center;

	//time measurement
	float time;
	float checkGestureTime = 0.f;

	// visualizing the hand on the mapped image frame
	ofImage handDepthImage;
	ofImage handColorImage;

	vector<std::array<float, 11 * 11>> mouseFeaturesReference;
	vector<std::array<float, 11 * 11>> videoFeaturesReference;
	vector<std::array<float, 11 * 11>> abortFeaturesReference;
	float mouseAccuracy = 0;
	float videoAccuracy = 0;
	float abortAccuracy = 0;

private:

	void addFeature(vector<std::array<float, 11 * 11>>& featuresReference, string item);
	//void initFingerCursor();
	//void updateFingerCursor();

};
