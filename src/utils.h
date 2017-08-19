#pragma once

#include "ofMain.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

class utils {

public:

	const static int utils::previewWidth = 640;
	const static int utils::previewHeight = 480;
	const static int utils::DEPTH_WIDTH = 512;
	const static int utils::DEPTH_HEIGHT = 424;
	const static int utils::WINDOW_WIDTH = 1600;
	const static int utils::WINDOW_HEIGHT = 900;
	const static int utils::DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

	static void removeSubstrs(string& s, string& p);
	static bool hasEnding(string const &fullString, string const &ending);

};