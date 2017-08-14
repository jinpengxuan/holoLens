#include "menu.h"

void menu::init() {
	//DWORD drives = GetLogicalDrives();
	//std::ostringstream stream;
	//stream << drives;
	//std::string str = stream.str();

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

	//build gui

	gui = new ofxDatGui( ofxDatGuiAnchor::TOP_LEFT );
	gui->addFRM();
	gui->addBreak()->setHeight(10.0f);
	
	openButton = gui->addButton("Open Videos");
	openButton->onButtonEvent(this, &menu::onButtonEvent);

	gui->addBreak()->setHeight(20.0f);

	upButton = gui->addButton("Up");
	upButton->onButtonEvent(this, &menu::onButtonEvent);

	gui->addBreak()->setHeight(10.0f);

	elements = 0;
	for (auto const& value : availableDrives) {
		vector<string> options;
		for (auto & entry : fs::directory_iterator(value)) {
			string entryStr = entry.path().string();
			if (entryStr.find(".") == std::string::npos) {
				options.push_back(entry.path().string());
			}
		}
		ofxDatGuiDropdown* dropDown = gui->addDropdown(value, options);
		dropDown->onDropdownEvent(this, &menu::onDropdownEvent);
		elements++;
	}
	gui->addBreak()->setHeight(10.0f);
	gui->addFooter();

}

void menu::loadSubOptions(string directory) {
	for (int i = 0; i < elements; i++) {
		gui->removeItem(4);
	}

	elements = 0;

	if (directory.length == 0) {
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
			vector<string> options;
			for (auto & entry : fs::directory_iterator(value)) {
				string entryStr = entry.path().string();
				if (entryStr.find(".") == std::string::npos) {
					options.push_back(entry.path().string());
				}
			}
			ofxDatGuiDropdown* dropDown = gui->addDropdown(value, options);
			dropDown->onDropdownEvent(this, &menu::onDropdownEvent);
			elements++;
		}
	}
	else {
		try {
			for (auto & entry1 : fs::directory_iterator(directory)) {
				string directory1 = entry1.path().string();
				try {
					vector<string> options;
					for (auto & entry2 : fs::directory_iterator(directory1)) {
						string entryStr2 = entry2.path().string();
						if (entryStr2.find(".") == std::string::npos) {
							options.push_back(entry2.path().string());
						}
					}
					ofxDatGuiDropdown* dropDown = gui->addDropdown(directory1, options);
					dropDown->onDropdownEvent(this, &menu::onDropdownEvent);
					elements++;
				}
				catch (const std::exception& e) {

				}
			}
		}
		catch (const std::exception& e) {

		}
	}

	gui->layoutGui();
}

void menu::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if ((e.target->getLabel()).compare("Up")) {

	}
	else {

	}
}

void menu::onDropdownEvent(ofxDatGuiDropdownEvent e)
{
	loadSubOptions(e.target->getLabel());
	gui->layoutGui(); // musste schnittstelle erweitern, da refresh methode nicht public war
}



void menu::draw() {
	
}