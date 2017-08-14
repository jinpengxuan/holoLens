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

	int elements = 0;

	std::array<string, 6> drives{ { "c:\\","d:\\","e:\\","f:\\","g:\\","h:\\" } };

	//gui object
	ofxDatGui* gui;

	//gui buttons
	ofxDatGuiButton* openButton;
	ofxDatGuiButton* upButton;

	ofxDatGuiLabel* pathLabel;

	//available drives list
	vector <string> availableDrives;

	//current parent folder
	string currentParent;

private:

	void loadSubOptions(string directory);
	void removeSubstrs(string& s, string& p);

	void onButtonEvent(ofxDatGuiButtonEvent e);
	void onDropdownEvent(ofxDatGuiDropdownEvent e);

};

