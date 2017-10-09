#pragma once

#include "ofxKinectForWindows2.h"
#include "applicationProperties.h"
#include "frame.h"
#include "imageUtils.h"
#include "stringUtils.h"
#include <limits>

class gestureTracker {

public:

	//lifecycle functions
	void init(vector<string> featureElements);
	void update();
	void draw();

	//capture the actual object frame in front of the camera
	void capture(string gestureType);

	//initialize the features of the captured images
	void initFeatures(vector<string> featureElements);

	//the kinect device object
	ofxKFW2::Device kinect;

	//flag
	bool featuresLoaded = false;

	//curser mode of the application
	applicationProperties::CursorMode cursorMode = applicationProperties::CursorMode::None;

	//coordinates of finger tips
	vector<ofVec3f> coordinateClusers;

	//central position
	ofVec3f center;

	//mapping coordinates from depth to color image and vice versa
	ICoordinateMapper * coordinateMapper;

	//time measurement
	float time;
	float checkGestureTime = 0.f;

	//features and accuracies for the gesture detection
	vector<std::array<float, 11 * 11>> mouseFeaturesReference;
	vector<std::array<float, 11 * 11>> videoFeaturesReference;
	vector<std::array<float, 11 * 11>> abortFeaturesReference;
	float mouseAccuracy = 0;
	float videoAccuracy = 0;
	float abortAccuracy = 0;

private:

	//add a feature to the item category
	void addFeature(vector<std::array<float, 11 * 11>>& featuresReference, string item);

	// visualizing the hand on the mapped image frame
	ofImage handImage;
	ofImage featureImage;
};
