#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include "appUtils.h"

class menuWrapper {

public:

	int elements = 0;

	bool collapsed = false;

	//gui panel objects
	ofxDatGui* fileSystemGui;
	ofxDatGui* gestureGui;
	ofxDatGui* framerateGui;
	ofxDatGui* sortingGui;

	//main menu options
	ofxDatGuiLabel* videoNameLabel;
	ofxDatGuiButton* collapseButton;
	ofxDatGuiButton* playButton;
	ofxDatGuiButton* evaluateButton;
	ofxDatGuiButton* learnMouseControlButton;
	ofxDatGuiButton* learnVideoControlButton;
	ofxDatGuiSlider* mouseControlPrecision;
	ofxDatGuiSlider* videoControlPrecision;

	//gui filesystem elements
	ofxDatGuiButton* openButton;
	ofxDatGuiButton* upButton;

	ofxDatGuiLabel* pathLabel;

	//gui sorting elements
	ofxDatGuiDropdown* sortOptions;

	//available drives list
	array<string, 6> drives;
	vector <string> availableDrives;

	template<typename T, typename args1, typename args2, class ListenerClass1, class ListenerClass2>
	void init(T* owner, void (ListenerClass1::*buttonListenerMethod)(args1), void (ListenerClass2::*dropDownListenerMethod)(args2)) {

		drives = { "c:\\","d:\\","e:\\","f:\\","g:\\","h:\\" };

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
		collapseButton->setStripeColor(ofColor::white);
		framerateGui->addBreak()->setHeight(10.0f);
		framerateGui->addFRM()->setStripeColor(ofColor::green);
		framerateGui->addBreak()->setHeight(10.0f);
		videoNameLabel = framerateGui->addLabel("");
		videoNameLabel->setBackgroundColor(ofColor(50.f, 1.f));
		videoNameLabel->setStripeVisible(false);
		framerateGui->addBreak()->setHeight(10.0f);
		playButton = framerateGui->addButton("Play");
		playButton->setStripeColor(ofColor::green);
		collapseButton->onButtonEvent(owner, buttonListenerMethod);
		playButton->onButtonEvent(owner, buttonListenerMethod);

		//Filesystem gui
		fileSystemGui = new ofxDatGui(ofxDatGuiAnchor::TOP_HALF_LEFT);
		(fileSystemGui->addLabel(":: File ::"))->setLabelAlignment(ofxDatGuiAlignment::CENTER);
		fileSystemGui->addBreak()->setHeight(10.0f);
		openButton = fileSystemGui->addButton("Open Videos");
		fileSystemGui->addBreak()->setHeight(10.0f);
		pathLabel = fileSystemGui->addLabel("");
		pathLabel->setBackgroundColor(ofColor(50.f, 1.f));
		pathLabel->setStripeVisible(false);
		fileSystemGui->addBreak()->setHeight(10.0f);
		upButton = fileSystemGui->addButton("Up");
		fileSystemGui->addBreak()->setHeight(10.0f);
		openButton->onButtonEvent(owner, buttonListenerMethod);
		upButton->onButtonEvent(owner, buttonListenerMethod);

		elements = 0;
		for (auto const& value : availableDrives) {
			ofxDatGuiButton* tempButton = fileSystemGui->addButton(value);
			tempButton->onButtonEvent(owner, buttonListenerMethod);
			tempButton->setStripeColor(ofColor::orange);
			elements++;
		}

		fileSystemGui->addFooter();
		fileSystemGui->getFooter()->setLabelWhenExpanded("");
		fileSystemGui->getFooter()->setLabelWhenCollapsed("");

		//Sorting gui

		sortingGui = new ofxDatGui(ofxDatGuiAnchor::TOP_HALF_RIGHT);
		(sortingGui->addLabel(":: Sorting ::"))->setLabelAlignment(ofxDatGuiAlignment::CENTER);
		sortingGui->addBreak()->setHeight(10.0f);
		vector<string> options = { "Length Ascending", "Length Descending", "Size Ascending", "Size Descending" };
		sortOptions = sortingGui->addDropdown("Sorting Options", options);
		sortOptions->onDropdownEvent(owner, dropDownListenerMethod);
		sortingGui->addBreak()->setHeight(10.0f);
		sortingGui->addFooter();
		sortingGui->getFooter()->setLabelWhenExpanded("");
		sortingGui->getFooter()->setLabelWhenCollapsed("");

		//Gesture GUI

		gestureGui = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
		(gestureGui->addLabel(":: Gestures ::"))->setLabelAlignment(ofxDatGuiAlignment::CENTER);
		gestureGui->addBreak()->setHeight(10.0f);
		evaluateButton = gestureGui->addButton("Evaluate");
		evaluateButton->onButtonEvent(owner, buttonListenerMethod);
		evaluateButton->setStripeColor(ofColor::green);
		gestureGui->addBreak()->setHeight(10.0f);
		gestureGui->addLabel("Mouse Control")->setStripeVisible(false);
		gestureGui->addBreak()->setHeight(10.0f);
		mouseControlPrecision = gestureGui->addSlider("Precision", 0.f, 100.f);
		mouseControlPrecision->setStripeColor(ofColor::blue);
		mouseControlPrecision->setValue(0.f);
		gestureGui->addBreak()->setHeight(10.0f);
		learnMouseControlButton = gestureGui->addButton("Learn");
		learnMouseControlButton->onButtonEvent(owner, buttonListenerMethod);
		learnMouseControlButton->setStripeColor(ofColor::blue);
		gestureGui->addBreak()->setHeight(10.0f);
		gestureGui->addLabel("Video Control")->setStripeVisible(false);
		gestureGui->addBreak()->setHeight(10.0f);
		videoControlPrecision = gestureGui->addSlider("Precision", 0.f, 100.f);
		videoControlPrecision->setStripeColor(ofColor::blueSteel);
		videoControlPrecision->setValue(0.f);
		gestureGui->addBreak()->setHeight(10.0f);
		learnVideoControlButton = gestureGui->addButton("Learn");
		learnVideoControlButton->onButtonEvent(owner, buttonListenerMethod);
		learnVideoControlButton->setStripeColor(ofColor::blueSteel);
		gestureGui->addFooter();
		gestureGui->getFooter()->setLabelWhenExpanded("");
		gestureGui->getFooter()->setLabelWhenCollapsed("");

	}

	template<typename T, typename args, class ListenerClass>
	void loadSubOptions(T* owner, void (ListenerClass::*listenerMethod)(args), string directory) {

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
				tempButton->onButtonEvent(owner, listenerMethod);
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
								string remove = stringUtils::hasEnding(directory, (string)"\\") ? directory : directory + "\\";
								stringUtils::removeSubstrs(directory1, remove);
								ofxDatGuiButton* tempButton = fileSystemGui->addButton(directory1);
								tempButton->onButtonEvent(owner, listenerMethod);
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

};