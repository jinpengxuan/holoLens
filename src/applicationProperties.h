#pragma once

#include "ofMain.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class applicationProperties {

public:

	//constant sizes
	const static int previewWidth = 640;
	const static int previewHeight = 480;
	const static int DEPTH_WIDTH = 512;
	const static int DEPTH_HEIGHT = 424;
	const static int WINDOW_WIDTH = 1600;
	const static int WINDOW_HEIGHT = 900;
	const static int HOG_SIZE = 96;
	const static int DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

	//enums of order, curser mode and media type of the images loaded
	const enum VideoOrder { LengthAsc, SizeAsc, LengthDesc, SizeDesc};
	const enum CursorMode { None, Pointer, Grab };
	const enum MediaType { Video, Feature };

};