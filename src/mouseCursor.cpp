#include "mouseCursor.h"

void mouseCursor::setup(vector<ofVec2f>& initCursorPos) {
	startPos = initCursorPos;
	actualPos = initCursorPos;
	cursorImage.load("cursor.png");
	initialized = true;
}

void mouseCursor::update(vector<ofVec2f>& actualCursorPos) {
	actualPos = actualCursorPos;
}

void mouseCursor::draw() {
	if (startPos.size() != actualPos.size())return;
	int count = 0;

	for (ofVec2f& pos : actualPos) {
		float actualX = pos.x - 100.f;
		float actualY = pos.y - 250.f;
		ofEnableAlphaBlending();
		//SetCursorPos(actualX, actualY);
		cursorImage.draw(actualX, actualY);
		ofDisableAlphaBlending();
		count++;
	}
}

void mouseCursor::tearDown() {
	cursorImage.clear();
	initialized = false;
}