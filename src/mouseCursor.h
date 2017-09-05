#pragma once

#include "ofMain.h"
#include "appUtils.h"
#include "imageUtils.h"

class mouseCursor {

public:

	void setup(vector<ofVec2f>& initCursorPos, appUtils::CursorMode cursorMode);
	void update(vector<ofVec2f>& actualCursorPos);
	void draw();
	void tearDown();

	bool initialized = false;
	appUtils::CursorMode currentCursorMode;

	ofImage normalFingerImage;
	ofImage specialFingerImage;
	vector<ofVec2f> startPos;
	vector<ofVec2f> actualPos;
	queue<ofVec2f> history;
	map<std::string, ofVec2f> fingerMap;

private:

};