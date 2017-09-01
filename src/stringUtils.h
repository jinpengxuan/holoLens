#pragma once

#include "ofMain.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class stringUtils {

public:

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