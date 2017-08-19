#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

	//ofSetWindowShape(previewWidth * 2, previewHeight * 2);
	ofSetWindowShape(utils::WINDOW_WIDTH, utils::WINDOW_HEIGHT);

	cam = ofEasyCam();
	cam.setPosition(ofVec3f(0, 0, 5));
	cam.lookAt(ofVec3f(0, 0, 0));

	ofSetBackgroundColor(ofColor(0));

	//get the starting time
	time = ofGetElapsedTimef();

	// set colors an positions for the lights and the material
	light.setDiffuseColor(ofColor(0.f, 255.f, 0.f));
	light.setAmbientColor(ofColor(255.f, 255.f, 255.f));
	light.setPosition(0, 0, -2000);

	//Gesture Init
	gestureTracker.init();

	//Menu Init
	//Check existing drives

	for (auto& path : drives) {
		try {
			for (auto & entry : fs::directory_iterator(path)) {
				break;
			}
			availableDrives.push_back(path);
		}
		catch (const std::exception& e) {

		}
	}

	//Filesystem gui

	fileSystemGui = new ofxDatGui(ofxDatGuiAnchor::TOP_LEFT);

	openButton = fileSystemGui->addButton("Open Videos");
	openButton->onButtonEvent(this, &ofApp::onButtonEvent);

	fileSystemGui->addBreak()->setHeight(40.0f);

	pathLabel = fileSystemGui->addLabel("");
	pathLabel->setBackgroundColor(ofColor(0.4f, 1.f));
	upButton = fileSystemGui->addButton("Up");
	upButton->onButtonEvent(this, &ofApp::onButtonEvent);

	fileSystemGui->addBreak()->setHeight(10.0f);

	elements = 0;
	for (auto const& value : availableDrives) {
		ofxDatGuiButton* tempButton = fileSystemGui->addButton(value);
		tempButton->onButtonEvent(this, &ofApp::onButtonEvent);
		elements++;
	}

	fileSystemGui->addFooter();

	//Framerate gui

	framerateGui = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	framerateGui->addFRM();

	fileSystemGui->setAutoDraw(false);
	framerateGui->setAutoDraw(false);

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
}

//--------------------------------------------------------------
void ofApp::draw() {

	cam.begin();

	videoContainer.draw();

	gestureTracker.draw();

	fileSystemGui->draw();
	framerateGui->draw();

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
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

	for (size_t i = 0; i < cloud.points.size(); ++i)
	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;*/
}

void ofApp::setVideoElements(string path) {
	videoElements.clear();
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
						if (utils::hasEnding(file, ".mkv") || utils::hasEnding(file, ".avi")) {
							videoElements.push_back(file);
						}
					}
				}
				else
				{
					//error
				}
			}
			catch (const std::exception& e) {

			}
		}
	}
	catch (const std::exception& e) {

	}
}

void ofApp::loadSubOptions(string directory) {

	for (int i = 0; i < elements; i++) {
		fileSystemGui->removeItem(5);
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
			catch (const std::exception& e) {

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
							string remove = utils::hasEnding(directory, "\\") ? directory : directory + "\\";
							utils::removeSubstrs(directory1, remove);
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
				catch (const std::exception& e) {

				}
			}
		}
		catch (const std::exception& e) {

		}
	}

	fileSystemGui->layoutGui();
}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if (e.target == openButton) {
		setVideoElements(pathLabel->getLabel());
		cout << "init videos" << endl;
		videoContainer.init(ofVec2f(0, 0), videoElements);
	}
	else if (e.target == upButton) {
		string parentLabel = pathLabel->getLabel();
		if (utils::hasEnding(parentLabel, ":\\")) {
			parentLabel = "";
			loadSubOptions(parentLabel);
			pathLabel->setLabel(parentLabel);
		}
		else {
			std::size_t found = parentLabel.find_last_of("/\\");
			utils::removeSubstrs(parentLabel, parentLabel.substr(found));
			if (utils::hasEnding(parentLabel, ":"))parentLabel += "\\";
			loadSubOptions(parentLabel);
			pathLabel->setLabel(parentLabel);
		}
	}
	else {
		string buttonLabel = e.target->getLabel();
		string path1 = (utils::hasEnding(pathLabel->getLabel(), "\\") || (pathLabel->getLabel()).length() == 0) ? pathLabel->getLabel() : (pathLabel->getLabel() + "\\");
		loadSubOptions(path1 + buttonLabel);
		pathLabel->setLabel(path1 + buttonLabel);
		fileSystemGui->layoutGui(); // musste schnittstelle erweitern, da refresh methode nicht public war
	}
}

void ofApp::onDropdownEvent(ofxDatGuiDropdownEvent e)
{

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
