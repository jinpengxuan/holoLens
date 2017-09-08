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
		//specialFingerImage.load("cursor_green.png");
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
		float actualX = actualPos.front().x * 6 - 100.f;
		float actualY = actualPos.front().y * 6;
		SetCursorPos(-ofGetWidth() * .2f + actualX,-ofGetHeight() * .5f + actualY);
		//normalFingerImage.draw(actualX, actualY);
	}
	else if (currentCursorMode == appUtils::CursorMode::Grab) {
		if (fingerMap.find("thumb") != fingerMap.end()) {
			drawMarker(thumbFingerImage, fingerMap["thumb"]);
		}
		if (fingerMap.find("otherFinger1") != fingerMap.end()) {
			drawMarker(normalFingerImage, fingerMap["otherFinger1"]);
		}
		if (fingerMap.find("otherFinger2") != fingerMap.end()) {
			drawMarker(normalFingerImage, fingerMap["otherFinger2"]);
		}
		if (fingerMap.find("otherFinger3") != fingerMap.end()) {
			drawMarker(normalFingerImage, fingerMap["otherFinger3"]);
		}
		if (fingerMap.find("otherFinger4") != fingerMap.end()) {
			drawMarker(normalFingerImage, fingerMap["otherFinger4"]);
		}
		//for(ofVec3f& point : actualPos){
		//	float actualX = point.x - 100.f;
		//	float actualY = -point.y + 250.f;
		//	normalFingerImage.draw(actualX, actualY);
		//}
		//}
	}
	ofDisableAlphaBlending();
}

void mouseCursor::tearDown() {
	specialFingerImage.clear();
	normalFingerImage.clear();
	initialized = false;
}

void mouseCursor::drawMarker(ofImage& image, ofVec3f& position) {
	float actualX = position.x - 100.f;
	float actualY = -position.y + 250.f;
	image.draw(actualX, actualY);
}