#pragma once

#include "ofMain.h"
#include "appUtils.h"
#include "stringUtils.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class pathUtils {

public:

	static void setPathElements(vector<string>& elements, string path, appUtils::MediaType type) {
		elements.clear();
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
							if (type == appUtils::MediaType::Video && (stringUtils::hasEnding(file, (string)".mkv") || stringUtils::hasEnding(file, (string)".avi"))) {
								elements.push_back(file);
							}
							else if (type == appUtils::MediaType::Feature && (stringUtils::hasEnding(file, (string)".png"))) {
								elements.push_back(file);
							}
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

};