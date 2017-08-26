#pragma once

#include "ofMain.h"

class mouseCursor {

public:

	void setup(ofVec2f initCursorPos);
	void update(ofVec2f actualCursorPos);
	void draw();
	void tearDown();

	bool initialized = false;

	ofImage cursorImage;
	ofVec2f startPos;
	ofVec2f actualPos;
	queue<ofVec2f> history;

private:

};