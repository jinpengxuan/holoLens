#pragma once

#include "ofMain.h"
#include "applicationProperties.h"
#include "imageUtils.h"
#include <deque>

class mouseCursor {

public:

	void setup(vector<ofVec3f>& initCursorPos, applicationProperties::CursorMode cursorMode);
	void update(vector<ofVec3f>& actualCursorPos);
	void draw();
	void tearDown();

	bool initialized = false;
	float rotationDegree = 0.f;
	float positionTrackingTime = 0.f;
	bool simulateMouseClick = false;
	bool dismissVideo = false;
	applicationProperties::CursorMode currentCursorMode = applicationProperties::CursorMode::None;


private:

	void drawMarker(ofImage& image, ofVec3f& position);
	void setNormalLine(array<ofVec2f, 2>& normal, map<string, ofVec3f>& fingerMap);
	void drawLine(array<ofVec2f, 2>& normal);
	void evaluateHistory();
	void simulateLeftMouseClick();

	array<ofVec2f, 2> initGrabHandNormal{ { ofVec2f(-1,-1), ofVec2f(-1,-1) } };
	array<ofVec2f, 2> grabHandNormal{ { ofVec2f(-1,-1), ofVec2f(-1,-1) } };

	vector<ofVec3f> startPos;
	vector<ofVec3f> actualPos;
	ofVec3f actualMousePosition;
	deque <ofVec3f> history;
	map<string, ofVec3f> fingerMap;

	ofImage pauseImage;
	ofImage playImage;
	ofImage doublePlayImage;
	ofImage rewindImage;
	ofImage doubleRewindImage;

	ofImage normalFingerImage;
	ofImage specialFingerImage;
	ofImage thumbFingerImage;
};