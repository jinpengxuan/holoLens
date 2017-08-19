#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

class menu {

public:

	void init();
	void draw();
	vector<string> getVideoPath();

	int elements = 0;
	bool isReady = false;

	std::array<string, 6> drives{ { "c:\\","d:\\","e:\\","f:\\","g:\\","h:\\" } };

	//gui panel objects
	ofxDatGui* fileSystemGui;
	ofxDatGui* framerateGui;

	//gui buttons
	ofxDatGuiButton* openButton;
	ofxDatGuiButton* upButton;

	ofxDatGuiLabel* pathLabel;

	string actualPath;

	//available drives list
	vector <string> availableDrives;

	vector<string> videoElements;

	//current parent folder
	string currentParent;

private:

	void loadSubOptions(string directory);
	void removeSubstrs(string& s, string& p);
	bool hasEnding(std::string const &fullString, std::string const &ending);
	void setVideoElements(string path);

	void onButtonEvent(ofxDatGuiButtonEvent e);
	void onDropdownEvent(ofxDatGuiDropdownEvent e);

};

