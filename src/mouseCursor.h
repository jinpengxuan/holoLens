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
	appUtils::CursorMode currentCursorMode;

	ofImage normalFingerImage;
	ofImage specialFingerImage;
	ofImage thumbFingerImage;
	vector<ofVec3f> startPos;
	vector<ofVec3f> actualPos;
	queue<ofVec3f> history;
	map<std::string, ofVec3f> fingerMap;

private:

};