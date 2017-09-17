#pragma once

#include "ofMain.h"
#include "applicationProperties.h"
#include "stringUtils.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class pathUtils {

public:

	static void setPathElements(vector<string>& elements, string path, applicationProperties::MediaType type) {
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
							if (type == applicationProperties::MediaType::Video 
								&& (stringUtils::hasEnding(file, (string)".mkv") || stringUtils::hasEnding(file, (string)".avi") || stringUtils::hasEnding(file, (string)".mp4"))) {
								elements.push_back(file);
							}
							else if (type == applicationProperties::MediaType::Feature && (stringUtils::hasEnding(file, (string)".png"))) {
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