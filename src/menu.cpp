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

	gui->addBreak()->setHeight(40.0f);

	pathLabel = gui->addLabel("");
	pathLabel->setBackgroundColor(ofColor(0.4f,1.f));
	upButton = gui->addButton("Up");
	upButton->onButtonEvent(this, &menu::onButtonEvent);

	gui->addBreak()->setHeight(10.0f);

	elements = 0;
	for (auto const& value : availableDrives) {
		ofxDatGuiButton* tempButton = gui->addButton(value);
		tempButton->onButtonEvent(this, &menu::onButtonEvent);
		elements++;
	}

	gui->addFooter();

	//setVideoElements("c:\\vids");
	//isReady = true;
}

void menu::loadSubOptions(string directory) {

	for (int i = 0; i < elements; i++) {
		gui->removeItem(7);
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
			ofxDatGuiButton* tempButton = gui->addButton(value);
			tempButton->onButtonEvent(this, &menu::onButtonEvent);
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
							string remove = hasEnding(directory,"\\") ? directory : directory + "\\";
							removeSubstrs(directory1, remove);
							ofxDatGuiButton* tempButton = gui->addButton(directory1);
							tempButton->onButtonEvent(this, &menu::onButtonEvent);
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

	gui->layoutGui();
}

vector<string> menu::getVideoPath() {
	vector<string> tempElements;
	if (!isReady)return tempElements;
	isReady = false;
	cout << "Get Videos" << endl;
	
	return videoElements;
}

void menu::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if (e.target == openButton) {
		cout << "Open Button" << endl;
		setVideoElements(pathLabel->getLabel());
		isReady = true;
	}
	else if (e.target == upButton) {
		string parentLabel = pathLabel->getLabel();
		if (hasEnding(parentLabel, ":\\")) {
			parentLabel = "";
			loadSubOptions(parentLabel);
			pathLabel->setLabel(parentLabel);
		}
		else {
			std::size_t found = parentLabel.find_last_of("/\\");
			removeSubstrs(parentLabel, parentLabel.substr(found));
			if (hasEnding(parentLabel, ":"))parentLabel += "\\";
			loadSubOptions(parentLabel);
			pathLabel->setLabel(parentLabel);
		}
	}
	else {
		string buttonLabel = e.target->getLabel();
		string path1 = (hasEnding(pathLabel->getLabel(), "\\")||(pathLabel->getLabel()).length()==0) ? pathLabel->getLabel() : (pathLabel->getLabel() + "\\");
		loadSubOptions(path1 + buttonLabel);
		pathLabel->setLabel(path1 + buttonLabel);
		gui->layoutGui(); // musste schnittstelle erweitern, da refresh methode nicht public war
	}
}

void menu::onDropdownEvent(ofxDatGuiDropdownEvent e)
{
	
}

void menu::draw() {
	
}

void menu::removeSubstrs(string& s, string& p) {
	string::size_type n = p.length();
	for (string::size_type i = s.find(p);
		i != string::npos;
		i = s.find(p))
		s.erase(i, n);
}

bool menu::hasEnding(std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
	}
	else {
		return false;
	}
}

void menu::setVideoElements(string path) {
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
						if (hasEnding(file, ".mkv") || hasEnding(file, ".avi")) {
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