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

		pauseImage.load("x0.png");
		playImage.load("x1.png");
		doublePlayImage.load("x2.png");
		rewindImage.load("-x1.png");
		doubleRewindImage.load("-x2.png");
		imageUtils::setFingerMap(fingerMap, initCursorPos);
		if (initGrabHandNormal[0].x == -1) {
			setNormalLine(initGrabHandNormal, fingerMap);
		}
		setNormalLine(grabHandNormal, fingerMap);
	}

	initialized = true;
}

void mouseCursor::update(vector<ofVec3f>& actualCursorPos) {
	actualPos = actualCursorPos;
	if (currentCursorMode == appUtils::CursorMode::Grab) {
		imageUtils::setFingerMap(fingerMap, actualCursorPos);
		if (initGrabHandNormal[0].x == -1) {
			setNormalLine(initGrabHandNormal, fingerMap);
		}
		setNormalLine(grabHandNormal, fingerMap);
		rotationDegree = imageUtils::getAngleBetweenVectors(initGrabHandNormal[1], grabHandNormal[1]);
		rotationDegree = rotationDegree*10.f / 2.f;
		//cout << rotationDegree << endl;
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
		if (fingerMap.find("firstOuterFinger") != fingerMap.end()) {
			drawMarker(specialFingerImage, fingerMap["firstOuterFinger"]);
		}
		if (fingerMap.find("secondOuterFinger") != fingerMap.end()) {
			drawMarker(specialFingerImage, fingerMap["secondOuterFinger"]);
		}
		if (fingerMap.find("otherFinger1") != fingerMap.end()) {
			drawMarker(normalFingerImage, fingerMap["otherFinger1"]);
		}
		if (fingerMap.find("otherFinger2") != fingerMap.end()) {
			drawMarker(normalFingerImage, fingerMap["otherFinger2"]);
		}
		drawLine(grabHandNormal);
		//for(ofVec3f& point : actualPos){
		//	float actualX = point.x - 100.f;
		//	float actualY = -point.y + 250.f;
		//	normalFingerImage.draw(actualX, actualY);
		//}
		//}
		ofVec2f imagePos = ofVec2f(-32,-ofGetHeight()/4);
		if (((int)rotationDegree) <= -2) {
			doubleRewindImage.draw(imagePos);
		}
		else if (((int)rotationDegree) == -1) {
			rewindImage.draw(imagePos);
		}
		else if (((int)rotationDegree) == 0) {
			pauseImage.draw(imagePos);
		}
		else if (((int)rotationDegree) == 1) {
			playImage.draw(imagePos);
		}
		else if (((int)rotationDegree) >= 2) {
			doublePlayImage.draw(imagePos);
		}
	}
	ofDisableAlphaBlending();
}

void mouseCursor::setNormalLine(array<ofVec2f, 2>& normal, map<string, ofVec3f>& fingerMap) {
	if (fingerMap.find("firstOuterFinger") != fingerMap.end() && fingerMap.find("secondOuterFinger") != fingerMap.end()) {
		float dx = fingerMap["secondOuterFinger"].x - fingerMap["firstOuterFinger"].x;
		float dy = fingerMap["secondOuterFinger"].y - fingerMap["firstOuterFinger"].y;

		normal[0] = ofVec2f(fingerMap["thumb"].x, fingerMap["thumb"].y);
		normal[1] = ofVec2f(-dy, dx);
	}
}

void mouseCursor::drawLine(array<ofVec2f, 2>& normal) {
	float actualX = normal[0].x - 100.f;
	float actualY = -normal[0].y + 250.f;
	float destX = (normal[0].x + normal[1].x) - 100.f;
	float destY = -(normal[0].y + normal[1].y) + 250.f;
	ofDrawLine(actualX, actualY, destX, destY);
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