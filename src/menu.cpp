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
		vector<string> options;
		for (auto & entry : fs::directory_iterator(value)) {
			string entryStr = entry.path().string();
			/*if (entryStr.find(".") == std::string::npos) {
				options.push_back(entry.path().string());
			}*/
			struct stat s;
			const char * c = entryStr.c_str();
			if (stat(c, &s) == 0)
			{
				if (s.st_mode & S_IFDIR)
				{
					removeSubstrs(entryStr, (string)value);
					options.push_back(entryStr);
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
		ofxDatGuiDropdown* dropDown = gui->addDropdown(value, options);
		dropDown->onDropdownEvent(this, &menu::onDropdownEvent);
		elements++;
	}
	gui->addBreak()->setHeight(10.0f);
	gui->addFooter();

}

void menu::loadSubOptions(string directory) {

	for (int i = 0; i < elements; i++) {
		gui->removeItem(7);
	}

	elements = 0;

	//ROOT
	if (directory.length() == 0) {
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
				struct stat s;
				const char * c = entryStr.c_str();
				if (stat(c, &s) == 0)
				{
					if (s.st_mode & S_IFDIR)
					{
						removeSubstrs(entryStr, (string)value);
						options.push_back(entryStr);
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
			ofxDatGuiDropdown* dropDown = gui->addDropdown(value, options);
			dropDown->onDropdownEvent(this, &menu::onDropdownEvent);
			elements++;
		}
	}
	//IN DIRECTORY
	else {
		try {
			for (auto & entry1 : fs::directory_iterator(directory)) {
				string directory1 = entry1.path().string();
				try {
					vector<string> options;
					for (auto & entry2 : fs::directory_iterator(directory1)) {
						string entryStr2 = entry2.path().string();
						struct stat s;
						const char * c = entryStr2.c_str();
						if (stat(c, &s) == 0)
						{
							if (s.st_mode & S_IFDIR)
							{
								removeSubstrs(entryStr2, directory1+"\\");
								options.push_back(entryStr2);
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
					struct stat s;
					const char * c = directory1.c_str();
					if (stat(c, &s) == 0)
					{
						if (s.st_mode & S_IFDIR)
						{
							removeSubstrs(directory1, directory+"\\");
							ofxDatGuiDropdown* dropDown = gui->addDropdown(directory1, options);
							dropDown->onDropdownEvent(this, &menu::onDropdownEvent);
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

void menu::onButtonEvent(ofxDatGuiButtonEvent e)
{
	if ((e.target->getLabel()).compare("Up")) {

	}
	else {

	}
}

void menu::onDropdownEvent(ofxDatGuiDropdownEvent e)
{
	cout << "Dropdown Event" << endl;
	string path1 = (pathLabel->getLabel()).length()>0 ? (pathLabel->getLabel()+"\\") : "";
	string path2 = (pathLabel->getLabel()).length()>0 ? (e.target->getName() + "\\") : e.target->getName();
	loadSubOptions(path1 + path2 + e.target->getLabel());
	pathLabel->setLabel(path1 + path2 + e.target->getLabel());
	gui->layoutGui(); // musste schnittstelle erweitern, da refresh methode nicht public war
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