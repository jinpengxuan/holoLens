#include "mouseCursor.h"

void mouseCursor::setup(vector<ofVec3f>& initCursorPos, appUtils::CursorMode cursorMode) {
	initialized = false;

	startPos = initCursorPos;
	actualPos = initCursorPos;
	currentCursorMode = cursorMode;

	if (currentCursorMode == appUtils::CursorMode::Pointer) {
		normalFingerImage.load("cursor.png");
	} 
	else if (currentCursorMode == appUtils::CursorMode::Grab) {
		normalFingerImage.load("cursor.png");
		specialFingerImage.load("cursor_green.png");
		thumbFingerImage.load("cursor_blue.png");
		imageUtils::setFingerMap(fingerMap, initCursorPos);
	}

	initialized = true;
}

void mouseCursor::update(vector<ofVec3f>& actualCursorPos) {
	actualPos = actualCursorPos;
	if (currentCursorMode == appUtils::CursorMode::Grab) {
		imageUtils::setFingerMap(fingerMap, actualCursorPos);
	}
}

void mouseCursor::draw() {
	//if (startPos.size() != actualPos.size())return;
	if (actualPos.size() == 0)return;
	ofEnableAlphaBlending();
	if (currentCursorMode == appUtils::CursorMode::Pointer) {
		float actualX = actualPos.front().x * 2 - 100.f;
		float actualY = -actualPos.front().y * 2 + 250.f;
		//SetCursorPos(actualX, actualY);
		normalFingerImage.draw(actualX, actualY);
	}
	else if (currentCursorMode == appUtils::CursorMode::Grab) {
		if (fingerMap.find("thumb") != fingerMap.end()) {
			float actualX = fingerMap["thumb"].x - 100.f;
			float actualY = -fingerMap["thumb"].y + 250.f;
			thumbFingerImage.draw(actualX, actualY);
		}
		if (fingerMap.find("firstOuterFinger") != fingerMap.end()) {
			float actualX = fingerMap["firstOuterFinger"].x - 100.f;
			float actualY = -fingerMap["firstOuterFinger"].y + 250.f;
			specialFingerImage.draw(actualX, actualY);
		}
		if (fingerMap.find("secondOuterFinger") != fingerMap.end()) {
			float actualX = fingerMap["secondOuterFinger"].x - 100.f;
			float actualY = -fingerMap["secondOuterFinger"].y + 250.f;
			specialFingerImage.draw(actualX, actualY);
		}
		if (fingerMap.find("otherFinger1") != fingerMap.end()) {
			float actualX = fingerMap["otherFinger1"].x - 100.f;
			float actualY = -fingerMap["otherFinger1"].y + 250.f;
			normalFingerImage.draw(actualX, actualY);
		}
		if (fingerMap.find("otherFinger2") != fingerMap.end()) {
			float actualX = fingerMap["otherFinger2"].x - 100.f;
			float actualY = -fingerMap["otherFinger2"].y + 250.f;
			normalFingerImage.draw(actualX, actualY);
		}
	}
	ofDisableAlphaBlending();
}

void mouseCursor::tearDown() {
	specialFingerImage.clear();
	normalFingerImage.clear();
	initialized = false;
}