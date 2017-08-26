#include "mouseCursor.h"

void mouseCursor::setup(ofVec2f initCursorPos) {
	startPos = initCursorPos;
	actualPos = initCursorPos;
	cursorImage.load("cursor.png");
	initialized = true;
}

void mouseCursor::update(ofVec2f actualCursorPos) {
	actualPos = actualCursorPos;
}

void mouseCursor::draw() {
	float actualX = (actualPos.x - startPos.x) * 2;
	float actualY = (actualPos.y - startPos.y) * 2;

	cursorImage.draw(actualX, actualY);
}

void mouseCursor::tearDown() {
	cursorImage.clear();
	initialized = false;
}