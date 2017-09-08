#pragma once

#include "ofMain.h"
#include "appUtils.h"
#include "imageUtils.h"

class mouseCursor {

public:

	void setup(vector<ofVec3f>& initCursorPos, appUtils::CursorMode cursorMode);
	void update(vector<ofVec3f>& actualCursorPos);
	void draw();
	void tearDown();

	bool initialized = false;
	float rotationDegree = 0.f;
	appUtils::CursorMode currentCursorMode;

	array<ofVec2f, 2> initGrabHandNormal{ { ofVec2f (-1,-1), ofVec2f(-1,-1) } };
	array<ofVec2f, 2> grabHandNormal{ { ofVec2f(-1,-1), ofVec2f(-1,-1) } };

	ofImage pauseImage;
	ofImage playImage;
	ofImage doublePlayImage;
	ofImage rewindImage;
	ofImage doubleRewindImage;

	ofImage normalFingerImage;
	ofImage specialFingerImage;
	ofImage thumbFingerImage;
	vector<ofVec3f> startPos;
	vector<ofVec3f> actualPos;
	queue<ofVec3f> history;
	map<string, ofVec3f> fingerMap;

private:

	void drawMarker(ofImage& image, ofVec3f& position);
	void setNormalLine(array<ofVec2f, 2>& normal, map<string, ofVec3f>& fingerMap);
	void drawLine(array<ofVec2f, 2>& normal);

};