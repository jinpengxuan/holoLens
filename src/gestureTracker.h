#pragma once

#include "ofxKinectForWindows2.h"
#include "appUtils.h"
#include <limits>

class gestureTracker {

public:
	
	void startDrag();
	void stopDrag();
	void init();
	void update();
	void draw();
	void capture();

	ofxKFW2::Device kinect;

	bool dragged = false;
	bool isCursor = false;
	ofVec2f cursorPosition;

	int trackingTime = 0;

	float rotationDegree = 0.f;
	int translation = 0;

	int minZ = numeric_limits<int>::max();

	int minX = numeric_limits<int>::max();
	int minY = numeric_limits<int>::max();
	int maxX = numeric_limits<int>::min();
	int maxY = numeric_limits<int>::min();

	int xShift = -200;
	int yShift = -400;
	int zShift = -300;

	//vector<ofVec2f> colorCoords;
	vector<ofVec3f> depthCoords;
	//vector<ofVec3f> drawingCoords;

	vector<ofVec3f> coordinateClusers;

	//central position
	ofVec3f center;

	//mapping coordinates from depth to color image and vice versa
	ICoordinateMapper * coordinateMapper;

	//time measurement
	float time;

	// visualizing the hand on the mapped image frame
	ofImage handImage;
	ofImage featureImage;

	float featuresReference[11 * 11];

private:

	//void initFingerCursor();
	//void updateFingerCursor();

};
