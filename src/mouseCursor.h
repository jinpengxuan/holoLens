#pragma once

#include "ofMain.h"
#include "applicationProperties.h"
#include "imageUtils.h"
#include <deque>

class mouseCursor {

public:

	//lifecycle functions
	void setup(vector<ofVec3f>& initCursorPos, applicationProperties::CursorMode cursorMode);
	void update(vector<ofVec3f>& actualCursorPos);
	void draw();

	//deallocate all objects of the cursor
	void tearDown();

	//flag
	bool initialized = false;

	//rotation degree for speed calculation
	float rotationDegree = 0.f;

	//time of the last position saved in the history
	float positionTrackingTime = 0.f;

	//simulate click event
	bool simulateMouseClick = false;

	//dismiss action tracked
	bool dismissVideo = false;

	//scaling calculation 
	float scaling = 1.f; // only positive values

	//cursor mode
	applicationProperties::CursorMode currentCursorMode = applicationProperties::CursorMode::None;


private:

	//drawing functions
	void drawMarker(ofImage& image, ofVec3f& position);
	void drawLine(array<ofVec2f, 2>& normal);

	//normal calculation
	void setNormalLine(array<ofVec2f, 2>& normal, map<string, ofVec3f>& fingerMap);
	
	//evaluate history and set actions
	void evaluateHistory();
	void simulateLeftMouseClick();

	//get distance of points to point cloud
	float getMeanDistance(ofVec3f& center, vector<ofVec3f>& cursorPosition);

	//start normal
	array<ofVec2f, 2> initGrabHandNormal{ { ofVec2f(-1,-1), ofVec2f(-1,-1) } };
	
	//actual normal
	array<ofVec2f, 2> grabHandNormal{ { ofVec2f(-1,-1), ofVec2f(-1,-1) } };

	//cursor positions
	vector<ofVec3f> startPos;
	vector<ofVec3f> actualPos;
	ofVec3f actualMousePosition;

	//initial distance of fingers to center
	float initialMeanDistanceToCenter;

	//position history
	deque <ofVec3f> history;

	//finger map to position
	map<string, ofVec3f> fingerMap;

	//play time images
	ofImage pauseImage;
	ofImage playImage;
	ofImage doublePlayImage;
	ofImage rewindImage;
	ofImage doubleRewindImage;

	//finger images
	ofImage normalFingerImage;
	ofImage specialFingerImage;
	ofImage thumbFingerImage;
};