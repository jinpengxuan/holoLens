#include "mouseCursor.h"

void mouseCursor::setup(vector<ofVec3f>& initCursorPos, applicationProperties::CursorMode cursorMode) {
	initialized = false;

	startPos = initCursorPos;
	actualPos = initCursorPos;
	currentCursorMode = cursorMode;

	if (currentCursorMode == applicationProperties::CursorMode::Pointer) {
		normalFingerImage.load("cursor.png");
	} 
	else if (currentCursorMode == applicationProperties::CursorMode::Grab) {
		normalFingerImage.load("cursor.png");
		specialFingerImage.load("cursor_green.png");
		thumbFingerImage.load("cursor_blue.png");

		pauseImage.load("x0.png");
		playImage.load("x1.png");
		doublePlayImage.load("x2.png");
		rewindImage.load("-x1.png");
		doubleRewindImage.load("-x2.png");

		imageUtils::setFingerMap(fingerMap, initCursorPos);
		ofVec3f centroid;
		imageUtils::setCentroid(startPos, centroid);
		initialMeanDistanceToCenter = getMeanDistance(centroid, startPos);

		setNormalLine(initGrabHandNormal, fingerMap);
		setNormalLine(grabHandNormal, fingerMap);
	}

	initialized = true;
}

void mouseCursor::update(vector<ofVec3f>& actualCursorPos) {
	if (currentCursorMode == applicationProperties::CursorMode::None) return;
	actualPos = actualCursorPos;

	if (ofGetElapsedTimeMillis() - positionTrackingTime >= 100) {
		positionTrackingTime = ofGetElapsedTimeMillis();

		ofVec3f centroid;
		imageUtils::setCentroid(actualPos, centroid);
		history.push_front(centroid);
		evaluateHistory();
	}

	if (currentCursorMode == applicationProperties::CursorMode::Grab) {
		ofVec3f centroid;
		imageUtils::setCentroid(startPos, centroid);
		float meanDistanceToCenter = getMeanDistance(centroid, actualPos);
		if (meanDistanceToCenter < 15.f)return;
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
	if (currentCursorMode == applicationProperties::CursorMode::Pointer) {
		/*float factorX = 1;
		float factorY = 1;
		if (history.size() >= 2) {
			factorX = abs(history.at(0).x - history.at(1).x);
			factorY = abs(history.at(0).y - history.at(1).y);
		}
		float factor = (factorX + factorY) / 40.f;
		float diffX = actualPos.front().x - startPos.front().x;
		float diffY = actualPos.front().y - startPos.front().y;*/

		actualMousePosition.x = -ofGetWidth() * .2f + actualPos.front().x * 5.f - 100.f;
		actualMousePosition.y = -ofGetHeight() * .5f + actualPos.front().y * 5.f;
		SetCursorPos(actualMousePosition.x, actualMousePosition.y);
		//normalFingerImage.draw(actualX, actualY);
	}
	else if (currentCursorMode == applicationProperties::CursorMode::Grab) {
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

		ofVec2f imagePos = ofVec2f(-32,-ofGetHeight()/4);
		if ((rotationDegree) <= -2) {
			doubleRewindImage.draw(imagePos);
		}
		else if ((rotationDegree) <= -1) {
			rewindImage.draw(imagePos);
		}
		else if ((rotationDegree) >= 1) {
			playImage.draw(imagePos);
		}
		else if ((rotationDegree) >= 2) {
			doublePlayImage.draw(imagePos);
		}
		else{
			pauseImage.draw(imagePos);
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
	thumbFingerImage.clear();
	pauseImage.clear();
	playImage.clear();
	doublePlayImage.clear();
	rewindImage.clear();
	doubleRewindImage.clear();
	history.clear();
	startPos.clear();
	actualPos.clear();
	fingerMap.clear();
	rotationDegree = 0;
	scaling = 0;
	initialized = false;
}

void mouseCursor::drawMarker(ofImage& image, ofVec3f& position) {
	float actualX = position.x - 100.f;
	float actualY = -position.y + 250.f;
	image.draw(actualX, actualY);
}

void mouseCursor::evaluateHistory() {
	if (currentCursorMode == applicationProperties::CursorMode::Pointer && history.size() == 20) {
		history.pop_back();
		deque <ofVec3f> ::iterator it;
		float x_move_start = history.begin()->x;
		float y_move_start = history.begin()->y;
		float x_move = 0;
		float y_move = 0;
		for (it = history.begin()+1; it != history.end(); ++it) {
			x_move += abs(it->x - x_move_start);
			y_move += abs(it->y - y_move_start);
		}
		x_move /= 20.f;
		y_move /= 20.f;
		float sumMove = x_move + y_move;
		//cout << sumMove << endl;
		if (sumMove < 20) {
			simulateLeftMouseClick();
			history.clear();
		}
	}
	else if (currentCursorMode == applicationProperties::CursorMode::Grab && history.size() == 10) {
		history.pop_back();
		deque <ofVec3f> ::iterator it;
		float x_move_start = history.begin()->x;
		float x_move_end = (history.end()-1)->x;

		ofVec3f centroid;
		imageUtils::setCentroid(actualPos, centroid);
		float meanDistanceToCenter = getMeanDistance(centroid, actualPos);
		if (initialMeanDistanceToCenter < meanDistanceToCenter) {
			scaling = 1.0 + (meanDistanceToCenter - initialMeanDistanceToCenter) / 10.f;
		}

		float x_move = x_move_end - x_move_start;
		if (x_move >= 50) {
			dismissVideo = true;
			cout << "dismiss" << endl;
			history.clear();
		}
	}
}

float mouseCursor::getMeanDistance(ofVec3f& center, vector<ofVec3f>& cursorPosition) {
	float xDistance = 0;
	float yDistance = 0;
	float meanDistance = 0;
	for (ofVec3f& pos : cursorPosition) {
		xDistance += abs(pos.x - center.x);
		yDistance += abs(pos.y - center.y);
		meanDistance = (xDistance + yDistance) / 2.f;
	}
	meanDistance /= 5.f;
	return meanDistance;
}

void mouseCursor::simulateLeftMouseClick() {

	simulateMouseClick = true;

	//POINT mouse;
	//GetCursorPos(&mouse);
	//HWND window = GetActiveWindow();

	//SendMessage(window, WM_LBUTTONDOWN, MK_LBUTTON, MAKELPARAM(mouse.x, mouse.y));
	//ofMouseEventArgs event = ofMouseEventArgs(ofMouseEventArgs::Type::Pressed, mouse.x, mouse.y, 0);
	//ofNotifyEvent(ofEvent<ofMouseEventArgs>(), event);
	//this_thread::sleep_for(std::chrono::milliseconds(500));
	//SendMessage(window, WM_LBUTTONUP, MK_LBUTTON, MAKELPARAM(mouse.x, mouse.y));
}