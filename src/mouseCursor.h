#pragma once

#include "ofMain.h"

class mouseCursor {

public:

	void setup(vector<ofVec2f>& initCursorPos);
	void update(vector<ofVec2f>& actualCursorPos);
	void draw();
	void tearDown();

	bool initialized = false;

	ofImage cursorImage;
	vector<ofVec2f> startPos;
	vector<ofVec2f> actualPos;
	queue<ofVec2f> history;

private:

};