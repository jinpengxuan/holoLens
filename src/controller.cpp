#include "controller.h"

//--------------------------------------------------------------
void controller::setup() {

	//ofSetWindowShape(previewWidth * 2, previewHeight * 2);
	//ofSetFrameRate(30);
	ofSetWindowShape(applicationProperties::WINDOW_WIDTH, applicationProperties::WINDOW_HEIGHT);

	cam = ofEasyCam();
	cam.setPosition(ofVec3f(0, 0, 500));
	cam.lookAt(ofVec3f(0, 0, 0));

	ofSetBackgroundColor(ofColor(0));

	//get the starting time
	time = ofGetElapsedTimef();

	// set colors an positions for the lights and the material
	light.setDiffuseColor(ofColor(0.f, 255.f, 0.f));
	light.setAmbientColor(ofColor(255.f, 255.f, 255.f));
	light.setPosition(0, 0, -2000);

	//Gesture Init
	vector<string> featureElements;
	pathUtils::setPathElements(featureElements, capturePath, applicationProperties::MediaType::Feature);
	gestureTracker.init(featureElements);

	//Menu Init
	menu.init(this, &controller::onButtonEvent, &controller::onDropdownEvent);
}

//--------------------------------------------------------------
void controller::update() {
	gestureTracker.update();
	videoContainer.update();

	menu.mouseControlPrecision->setValue(gestureTracker.mouseAccuracy);
	menu.videoControlPrecision->setValue(gestureTracker.videoAccuracy);
	menu.abortControlPrecision->setValue(gestureTracker.abortAccuracy);

	if (gestureTracker.cursorMode == applicationProperties::CursorMode::None) {
		if (mouseCursor.initialized) {
			menu.videoControlLabel->setBackgroundColor(ofColor(25.f, 1.f));
			menu.mouseControlLabel->setBackgroundColor(ofColor(25.f, 1.f));
			mouseCursor.tearDown();
		}
		return;
	}
	else if (gestureTracker.cursorMode == applicationProperties::CursorMode::Pointer) {
		if (!mouseCursor.initialized || mouseCursor.currentCursorMode == applicationProperties::CursorMode::Grab) {
			mouseCursor.tearDown();
			menu.videoControlLabel->setBackgroundColor(ofColor(25.f, 1.f));
			menu.mouseControlLabel->setBackgroundColor(ofColor::forestGreen);
			mouseCursor.setup(gestureTracker.coordinateClusers, gestureTracker.cursorMode);
		}
		else {
			mouseCursor.update(gestureTracker.coordinateClusers);
		}
		if (mouseCursor.simulateMouseClick) {
			controller::mousePressed(ofGetMouseX(), ofGetMouseY(), 0);
			mouseCursor.simulateMouseClick = false;
			//ofApp::mouseReleased(ofGetMouseX(), ofGetMouseY(), 0);
		}
	}
	else if (gestureTracker.cursorMode == applicationProperties::CursorMode::Grab) {
		if (!mouseCursor.initialized || mouseCursor.currentCursorMode == applicationProperties::CursorMode::Pointer) {
			mouseCursor.tearDown();
			menu.videoControlLabel->setBackgroundColor(ofColor::forestGreen);
			menu.mouseControlLabel->setBackgroundColor(ofColor(25.f, 1.f));
			mouseCursor.setup(gestureTracker.coordinateClusers, gestureTracker.cursorMode);
		}
		else {
			mouseCursor.update(gestureTracker.coordinateClusers);
		}

		videoContainer.scaling = mouseCursor.scaling;

		if ((int)mouseCursor.rotationDegree == 0) {
			videoContainer.setSpeed(0);
			videoContainer.pause(true);

		}
		else {
			videoContainer.setSpeed((int)mouseCursor.rotationDegree);
			videoContainer.pause(false);
		}
		if (mouseCursor.dismissVideo) {
			videoContainer.dismissVideo();
			mouseCursor.dismissVideo=false;
		}
	}
	
	//if mouse mode is activated by gesture tracker, manipulate mouse position and clicks
	// SetCursorPos(100,100);
}

//--------------------------------------------------------------
void controller::draw() {

	cam.begin();

	videoContainer.draw();

	gestureTracker.draw();

	if (mouseCursor.initialized) {
		mouseCursor.draw();
	}

	cam.end();
}

void controller::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if (e.target == menu.collapseButton) {
		if (menu.collapsed) {
			menu.collapsed = false;
			menu.fileSystemGui->expand();
			menu.sortingGui->expand();
			menu.gestureGui->expand();
			menu.collapseButton->setLabel("-");
		}
		else {
			menu.collapsed = true;
			menu.fileSystemGui->collapse();
			menu.sortingGui->collapse();
			menu.gestureGui->collapse();
			menu.collapseButton->setLabel("+");
		}
	} 
	else if(e.target == menu.playButton) {
		if (menu.playButton->getLabel() == "Play") {
			videoContainer.pause(false);
			menu.playButton->setLabel("Pause");
		}
		else {
			videoContainer.pause(true);
			menu.playButton->setLabel("Play");
		}
	}
	else if (e.target == menu.evaluateButton) {
		vector<string> featureElements;
		pathUtils::setPathElements(featureElements, capturePath, applicationProperties::MediaType::Feature);
		gestureTracker.initFeatures(featureElements);
	}
	else if (e.target == menu.learnMouseControlButton) {
		gestureTracker.capture("mouse");
	}
	else if (e.target == menu.learnVideoControlButton) {
		gestureTracker.capture("video");
	}
	else if (e.target == menu.learnAbortControlButton) {
		gestureTracker.capture("abort");
	}
	else if (e.target == menu.openButton) {
		pathUtils::setPathElements(videoElements, menu.pathLabel->getLabel(), applicationProperties::MediaType::Video);
		cout << "init videos" << endl;
		videoContainer.init(ofVec2f(0, 0), videoElements);
		menu.videoNameLabel->setLabel(videoContainer.videoName);
	}
	else if (e.target == menu.upButton) {
		string parentLabel = menu.pathLabel->getLabel();
		if (parentLabel.length() == 0)return;
		if (stringUtils::hasEnding(parentLabel, (string)":\\")) {
			parentLabel = "";
			menu.loadSubOptions(this, &controller::onButtonEvent,parentLabel);
			menu.pathLabel->setLabel(parentLabel);
		}
		else {
			size_t found = parentLabel.find_last_of("/\\");
			stringUtils::removeSubstrs(parentLabel, parentLabel.substr(found));
			if (stringUtils::hasEnding(parentLabel, (string)":"))parentLabel += "\\";
			menu.loadSubOptions(this, &controller::onButtonEvent, parentLabel);
			menu.pathLabel->setLabel(parentLabel);
		}
	}
	else {
		string buttonLabel = e.target->getLabel();
		string path1 = (stringUtils::hasEnding(menu.pathLabel->getLabel(), (string)"\\") 
			|| (menu.pathLabel->getLabel()).length() == 0) ? menu.pathLabel->getLabel() : (menu.pathLabel->getLabel() + "\\");
		menu.loadSubOptions(this, &controller::onButtonEvent, path1 + buttonLabel);
		menu.pathLabel->setLabel(path1 + buttonLabel);
		menu.fileSystemGui->layoutGui(); // upgraded gui to get a refresh functionality after deletion
	}
}

void controller::onDropdownEvent(ofxDatGuiDropdownEvent e)
{
	if (e.target->getLabel() == "Length Ascending") {
		videoContainer.reorderVideos(applicationProperties::VideoOrder::LengthAsc);
	} else if (e.target->getLabel() == "Size Ascending") {
		videoContainer.reorderVideos(applicationProperties::VideoOrder::SizeAsc);
	}
	else if (e.target->getLabel() == "Length Descending") {
		videoContainer.reorderVideos(applicationProperties::VideoOrder::LengthDesc);
	}
	else if (e.target->getLabel() == "Size Descending") {
		videoContainer.reorderVideos(applicationProperties::VideoOrder::SizeDesc);
	}
	menu.videoNameLabel->setLabel(videoContainer.videoName);
}

//--------------------------------------------------------------
void controller::keyPressed(int key) {

}

//--------------------------------------------------------------
void controller::keyReleased(int key) {

}

//--------------------------------------------------------------
void controller::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void controller::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void controller::mousePressed(int x, int y, int button) {
	cout << "mouse pressed x: " << x << " y:" << y << " button:" << button <<  endl;
}

//--------------------------------------------------------------
void controller::mouseReleased(int x, int y, int button) {
	cout << "mouse released" << endl;
}

//--------------------------------------------------------------
void controller::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void controller::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void controller::dragEvent(ofDragInfo dragInfo) {

}
