#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

	//ofSetWindowShape(previewWidth * 2, previewHeight * 2);
	ofSetWindowShape(appUtils::WINDOW_WIDTH, appUtils::WINDOW_HEIGHT);

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
	setPathElements(featureElements, capturePath, MediaType::Feature);
	gestureTracker.init(featureElements);

	//Menu Init
	//Check existing drives

	for (auto& path : drives) {
		try {
			for (auto & entry : fs::directory_iterator(path)) {
				break;
			}
			availableDrives.push_back(path);
		}
		catch (const exception& e) {

		}
	}

	//Framerate gui

	framerateGui = new ofxDatGui(ofxDatGuiAnchor::TOP_LEFT);
	collapseButton = framerateGui->addButton("-");
	collapseButton->setLabelAlignment(ofxDatGuiAlignment::CENTER);
	collapseButton->onButtonEvent(this, &ofApp::onButtonEvent);
	collapseButton->setStripeColor(ofColor::white);
	framerateGui->addBreak()->setHeight(10.0f);
	framerateGui->addFRM()->setStripeColor(ofColor::green);
	framerateGui->addBreak()->setHeight(10.0f);
	videoNameLabel = framerateGui->addLabel("");
	videoNameLabel->setBackgroundColor(ofColor(50.f, 1.f));
	videoNameLabel->setStripeVisible(false);
	framerateGui->addBreak()->setHeight(10.0f);
	playButton = framerateGui->addButton("Play");
	playButton->onButtonEvent(this, &ofApp::onButtonEvent);
	playButton->setStripeColor(ofColor::green);
	framerateGui->addBreak()->setHeight(10.0f);
	evaluateButton = framerateGui->addButton("Evaluate");
	evaluateButton->onButtonEvent(this, &ofApp::onButtonEvent);
	evaluateButton->setStripeColor(ofColor::green);
	framerateGui->addBreak()->setHeight(10.0f);
	framerateGui->addLabel("Mouse Control")->setStripeVisible(false);
	framerateGui->addBreak()->setHeight(10.0f);
	mouseControlPrecision = framerateGui->addSlider("Precision", 0.f, 100.f);
	mouseControlPrecision->setStripeColor(ofColor::blue);
	mouseControlPrecision->setValue(0.f);
	framerateGui->addBreak()->setHeight(10.0f);
	learnMouseControlButton = framerateGui->addButton("Learn");
	learnMouseControlButton->onButtonEvent(this, &ofApp::onButtonEvent);
	learnMouseControlButton->setStripeColor(ofColor::blue);
	framerateGui->addBreak()->setHeight(10.0f);
	framerateGui->addLabel("Video Control")->setStripeVisible(false);
	framerateGui->addBreak()->setHeight(10.0f);
	videoControlPrecision = framerateGui->addSlider("Precision", 0.f, 100.f);
	videoControlPrecision->setStripeColor(ofColor::blueSteel);
	videoControlPrecision->setValue(0.f);
	framerateGui->addBreak()->setHeight(10.0f);
	learnVideoControlButton = framerateGui->addButton("Learn");
	learnVideoControlButton->onButtonEvent(this, &ofApp::onButtonEvent);
	learnVideoControlButton->setStripeColor(ofColor::blueSteel);
	framerateGui->addBreak()->setHeight(10.0f);

	//Filesystem gui

	fileSystemGui = new ofxDatGui(framerateGui->getWidth() + 1, 0);

	(fileSystemGui->addLabel(":: Directory Selector ::"))->setLabelAlignment(ofxDatGuiAlignment::CENTER);
	fileSystemGui->addBreak()->setHeight(10.0f);
	openButton = fileSystemGui->addButton("Open Videos");
	openButton->onButtonEvent(this, &ofApp::onButtonEvent);
	fileSystemGui->addBreak()->setHeight(10.0f);
	pathLabel = fileSystemGui->addLabel("");
	pathLabel->setBackgroundColor(ofColor(50.f, 1.f));
	pathLabel->setStripeVisible(false);
	fileSystemGui->addBreak()->setHeight(10.0f);
	upButton = fileSystemGui->addButton("Up");
	upButton->onButtonEvent(this, &ofApp::onButtonEvent);

	fileSystemGui->addBreak()->setHeight(10.0f);

	elements = 0;
	for (auto const& value : availableDrives) {
		ofxDatGuiButton* tempButton = fileSystemGui->addButton(value);
		tempButton->onButtonEvent(this, &ofApp::onButtonEvent);
		tempButton->setStripeColor(ofColor::orange);
		elements++;
	}

	fileSystemGui->addFooter();
	fileSystemGui->getFooter()->setLabelWhenExpanded("");
	fileSystemGui->getFooter()->setLabelWhenCollapsed("");

	//Sorting gui
	sortingGui = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	(sortingGui->addLabel(":: Sorting ::"))->setLabelAlignment(ofxDatGuiAlignment::CENTER);
	sortingGui->addBreak()->setHeight(10.0f);
	vector<string> options = {"Length Ascending", "Length Descending", "Size Ascending", "Size Descending" };
	sortOptions = sortingGui->addDropdown("Sorting Options", options);
	sortOptions->onDropdownEvent(this, &ofApp::onDropdownEvent);
	sortingGui->addBreak()->setHeight(10.0f);
	sortingGui->addFooter();
	sortingGui->getFooter()->setLabelWhenExpanded("");
	sortingGui->getFooter()->setLabelWhenCollapsed("");

	//setVideoElements("c:\\vids");
	//isReady = true;

	//ofVec3f pos = ofVec3f(0,0,-1);
	//ofVec3f target = ofVec3f(0, 0, 0);

	//cam = ofCamera();
	//cam.lookAt(target);
	//cam.setPosition(pos);
}

//--------------------------------------------------------------
void ofApp::update() {
	gestureTracker.update();
	videoContainer.update();

	float mousePrecision = gestureTracker.mouseAccuracy;
	float videoPrecision = gestureTracker.videoAccuracy;

	if (mousePrecision == numeric_limits<int>::max() && mousePrecision == numeric_limits<int>::max()) return;

	mousePrecision = (2000.f - mousePrecision) / 1000.f * 100.f;
	videoPrecision = (2000.f - videoPrecision) / 1000.f * 100.f;

	mousePrecision = mousePrecision < 0 ? 0 : (mousePrecision > 100.f  ? 100.f : mousePrecision);
	videoPrecision = videoPrecision < 0 ? 0 : (videoPrecision > 100.f ? 100.f : videoPrecision);

	if (mousePrecision > 50) {
		if (!mouseCursor.initialized) {
			mouseCursor.setup(ofVec2f(gestureTracker.cursorPosition.x, gestureTracker.cursorPosition.y));
		}
		else {
			mouseCursor.update(ofVec2f(gestureTracker.cursorPosition.x, gestureTracker.cursorPosition.y));
		}
	}

	mouseControlPrecision->setValue(mousePrecision);
	videoControlPrecision->setValue(videoPrecision);
	
	//if mouse mode is activated by gesture tracker, manipulate mouse position and clicks
	// SetCursorPos(100,100);
}

//--------------------------------------------------------------
void ofApp::draw() {

	cam.begin();

	videoContainer.draw();

	gestureTracker.draw();

	if (mouseCursor.initialized) {
		mouseCursor.draw();
	}

	cam.end();
}

void ofApp::testPCL() {
	/*pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
	cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
	cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
	cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << endl;

	for (size_t i = 0; i < cloud.points.size(); ++i)
	cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << endl;*/
}

void ofApp::setPathElements(vector<string>& elements, string path, MediaType type) {
	elements.clear();
	try {
		for (auto & entry1 : fs::directory_iterator(path)) {
			string file = entry1.path().string();
			try {
				struct stat s;
				const char * c = file.c_str();
				if (stat(c, &s) == 0)
				{
					if (s.st_mode & S_IFDIR)
					{
						//directory
					}
					else if (s.st_mode & S_IFREG)
					{
						if (type == MediaType::Video && (appUtils::hasEnding(file, (string)".mkv") || appUtils::hasEnding(file, (string)".avi"))) {
							elements.push_back(file);
						} else if (type == MediaType::Feature && (appUtils::hasEnding(file, (string)".png"))) {
							elements.push_back(file);
						}
					}
				}
				else
				{
					//error
				}
			}
			catch (const exception& e) {

			}
		}
	}
	catch (const exception& e) {

	}
}

void ofApp::loadSubOptions(string directory) {

	for (int i = 0; i < elements; i++) {
		fileSystemGui->removeItem(8);
	}

	elements = 0;

	//ROOT
	if (directory.length() == 0) {
		availableDrives.clear();
		for (auto& path : drives) {
			try {
				for (auto & entry : fs::directory_iterator(path)) {
					break;
				}
				availableDrives.push_back(path);
			}
			catch (const exception& e) {

			}
		}

		for (auto const& value : availableDrives) {
			ofxDatGuiButton* tempButton = fileSystemGui->addButton(value);
			tempButton->onButtonEvent(this, &ofApp::onButtonEvent);
			elements++;
		}
	}
	//IN DIRECTORY
	else {
		try {
			for (auto & entry1 : fs::directory_iterator(directory)) {
				string directory1 = entry1.path().string();
				try {
					struct stat s;
					const char * c = directory1.c_str();
					if (stat(c, &s) == 0)
					{
						if (s.st_mode & S_IFDIR)
						{
							string remove = appUtils::hasEnding(directory, (string)"\\") ? directory : directory + "\\";
							appUtils::removeSubstrs(directory1, remove);
							ofxDatGuiButton* tempButton = fileSystemGui->addButton(directory1);
							tempButton->onButtonEvent(this, &ofApp::onButtonEvent);
							elements++;
						}
						else if (s.st_mode & S_IFREG)
						{
							//file
						}
					}
					else
					{
						//error
					}
				}
				catch (const exception& e) {

				}
			}
		}
		catch (const exception& e) {

		}
	}

	fileSystemGui->layoutGui();
}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if (e.target == collapseButton) {
		if (collapsed) {
			collapsed = false;
			fileSystemGui->expand();
			sortingGui->expand();
			collapseButton->setLabel("-");
		}
		else {
			collapsed = true;
			fileSystemGui->collapse();
			sortingGui->collapse();
			collapseButton->setLabel("+");
		}
	} 
	else if(e.target == playButton) {
		if (playButton->getLabel() == "Play") {
			videoContainer.pause(false);
			playButton->setLabel("Pause");
		}
		else {
			videoContainer.pause(true);
			playButton->setLabel("Play");
		}
	}
	else if (e.target == evaluateButton) {
		vector<string> featureElements;
		setPathElements(featureElements, capturePath, MediaType::Feature);
		gestureTracker.initFeatures(featureElements);
	}
	else if (e.target == learnMouseControlButton) {
		gestureTracker.capture("mouse");
	}
	else if (e.target == learnVideoControlButton) {
		gestureTracker.capture("video");
	}
	else if (e.target == openButton) {
		setPathElements(videoElements, pathLabel->getLabel(), MediaType::Video);
		cout << "init videos" << endl;
		videoContainer.init(ofVec2f(0, 0), videoElements);
		videoNameLabel->setLabel(videoContainer.videoName);
	}
	else if (e.target == upButton) {
		string parentLabel = pathLabel->getLabel();
		if (parentLabel.length() == 0)return;
		if (appUtils::hasEnding(parentLabel, (string)":\\")) {
			parentLabel = "";
			loadSubOptions(parentLabel);
			pathLabel->setLabel(parentLabel);
		}
		else {
			size_t found = parentLabel.find_last_of("/\\");
			appUtils::removeSubstrs(parentLabel, parentLabel.substr(found));
			if (appUtils::hasEnding(parentLabel, (string)":"))parentLabel += "\\";
			loadSubOptions(parentLabel);
			pathLabel->setLabel(parentLabel);
		}
	}
	else {
		string buttonLabel = e.target->getLabel();
		string path1 = (appUtils::hasEnding(pathLabel->getLabel(), (string)"\\") || (pathLabel->getLabel()).length() == 0) ? pathLabel->getLabel() : (pathLabel->getLabel() + "\\");
		loadSubOptions(path1 + buttonLabel);
		pathLabel->setLabel(path1 + buttonLabel);
		fileSystemGui->layoutGui(); // musste schnittstelle erweitern, da refresh methode nicht public war
	}
}

void ofApp::onDropdownEvent(ofxDatGuiDropdownEvent e)
{
	if (!videoContainer.readyState)return;
	if (e.target->getLabel() == "Length Ascending") {
		videoContainer.reorderVideos(appUtils::VideoOrder::LengthAsc);
	} else if (e.target->getLabel() == "Size Ascending") {
		videoContainer.reorderVideos(appUtils::VideoOrder::SizeAsc);
	}
	else if (e.target->getLabel() == "Length Descending") {
		videoContainer.reorderVideos(appUtils::VideoOrder::LengthDesc);
	}
	else if (e.target->getLabel() == "Size Descending") {
		videoContainer.reorderVideos(appUtils::VideoOrder::SizeDesc);
	}
	videoNameLabel->setLabel(videoContainer.videoName);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
