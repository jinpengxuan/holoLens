#pragma once

#include "ofMain.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class appUtils {

public:

	const static int previewWidth = 640;
	const static int previewHeight = 480;
	const static int DEPTH_WIDTH = 512;
	const static int DEPTH_HEIGHT = 424;
	const static int WINDOW_WIDTH = 1600;
	const static int WINDOW_HEIGHT = 900;
	const static int DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

	const enum VideoOrder { Length, Size };

	static void removeSubstrs(string& inputString, string& pattern) {
		string::size_type n = pattern.length();
		for (string::size_type i = inputString.find(pattern);
			i != string::npos;
			i = inputString.find(pattern))
			inputString.erase(i, n);
	}

	static bool hasEnding(string &fullString, string &ending) {
		if (fullString.length() >= ending.length()) {
			return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
		}
		else {
			return false;
		}
	}

};