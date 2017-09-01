#pragma once

#include "ofMain.h"
#include "frame.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class imageUtils {

public:

	bool sortVecByDepth(ofVec3f i, ofVec3f j) { return (i.z < j.z); }

	static void setHandImage(ofImage& handImage, frame frame) {

		//we need a frame that is equal in height and width
		if (frame.widthImg > frame.heightImg) {
			float difference = frame.widthImg - frame.heightImg;
			frame.minYImg = frame.minYImg - difference / 2;
			frame.maxYImg = frame.maxYImg + difference / 2;
			frame.heightImg = frame.widthImg;
		}
		else {
			float difference = frame.heightImg - frame.widthImg;
			frame.minXImg = frame.minXImg - difference / 2;
			frame.maxXImg = frame.maxXImg + difference / 2;
			frame.widthImg = frame.heightImg;
		}

		handImage.allocate(frame.widthImg, frame.heightImg, OF_IMAGE_GRAYSCALE);
		ofPixels pix = ofPixels();

		float depthAmount = (frame.maxZ - frame.minZ);

		pix.allocate(frame.widthImg, frame.heightImg, OF_IMAGE_GRAYSCALE);
		pix.setColor(0);

		for (int y = frame.minYImg; y < frame.maxYImg; y++) {
			for (int x = frame.minXImg; x < frame.maxXImg; x++) {
				int index = x + y*frame.width;
				int distance = frame.pixels[index];

				int thumbIndex = (x - frame.minXImg) + (y - frame.minYImg) * frame.widthImg;
				float greyVal = (frame.maxZ - distance) / depthAmount * 255.f;
				pix.setColor(thumbIndex, ofColor(greyVal < 0 ? 0 : greyVal));
			}
		}

		handImage.setFromPixels(pix);
		handImage.resize(appUtils::HOG_SIZE, appUtils::HOG_SIZE);
	}

	static void setDepthCoordinatesSorted(vector<ofVec3f>& depthCoordinates, frame& frame) {
		depthCoordinates.clear();

		float difference = 80;

		int minX = numeric_limits<int>::max();
		int minY = numeric_limits<int>::max();

		int maxX = numeric_limits<int>::min();
		int maxY = numeric_limits<int>::min();

		for (int y = 0; y < frame.height; y++) {
			for (int x = 0; x < frame.width; x++) {
				int index = x + y*frame.width;
				int distance = frame.pixels[index];

				if (distance > frame.maxZ || distance < frame.minZ)continue;

				minX = x < minX ? x : minX;
				maxX = x > maxX ? x : maxX;
				minY = y < minY ? y : minY;
				maxY = y > maxY ? y : maxY;

				depthCoordinates.push_back(ofVec3f(x, y, distance));
			}
		}

		frame.minXImg = minX;
		frame.maxXImg = maxX;
		frame.minYImg = minY;
		frame.maxYImg = maxY;
		frame.widthImg = maxX - minX;
		frame.heightImg = maxY - minY;
	}

	static void setFrame(frame& frameObj, const ofShortPixels& pixels) {
		int minZ = numeric_limits<int>::max();
		int length = frameObj.width*frameObj.height;
		frameObj.width = frameObj.maxX - frameObj.minX;
		frameObj.height = frameObj.maxY - frameObj.minY;
		frameObj.pixels = new int[length];

		for (int y = frameObj.minY; y < frameObj.maxY; y ++) {
			for (int x = frameObj.minX; x < frameObj.maxX; x ++) {
				int index = x + y*pixels.getWidth();
				int indexFrame = (x - frameObj.minX) + (y - frameObj.minY)*frameObj.width;
				int distance = pixels[index];
				if (distance < 400 || distance > 700) continue;

				// Outlier Removal

				int boundaryMax = distance + 10;
				int boundaryMin = distance - 10;
				int indexTop = index - pixels.getWidth();
				int indexBottom = index + pixels.getWidth();
				int indexLeft = index - 1;
				int indexRight = index + 1;
				if (pixels[indexTop] < (boundaryMin) || pixels[indexTop] > (boundaryMax) ||
					pixels[indexBottom] < (boundaryMin) || pixels[indexBottom] > (boundaryMax) ||
					pixels[indexLeft] < (boundaryMin) || pixels[indexLeft] > (boundaryMax) ||
					pixels[indexRight] < (boundaryMin) || pixels[indexRight] > (boundaryMax)) {
					frameObj.pixels[indexFrame] = (pixels[indexTop] + pixels[indexBottom] + pixels[indexLeft] + pixels[indexRight]) / 4;
				}
				else {
					frameObj.pixels[indexFrame] = distance;
					minZ = distance < minZ ? distance : minZ;
				}
			}
		}
		frameObj.minZ = minZ;
		frameObj.maxZ = minZ + 80.f;
	}

	static void setFeatureVector(const ofPixels &pixels, std::array<float, 11 * 11> &features) {

		const int width = appUtils::HOG_SIZE;
		const int height = appUtils::HOG_SIZE;
		const int size = appUtils::HOG_SIZE * appUtils::HOG_SIZE;
		if(pixels.size() != size) return;

		int Ix[size];
		int Iy[size];

		for (int y = 0; y < height ; y++) {
			for (int x = 0; x < width ; x++) {
				int pos = y * width + x;

				if (x == 0) {
					Ix[pos] = getGradientX(pixels[pos], pixels[pos + 1]);
				}
				else if (x == width - 1) {
					Ix[pos] = getGradientX(pixels[pos - 1], pixels[pos]);
				}
				else {
					Ix[pos] = getGradientX(pixels[pos - 1], pixels[pos + 1]);
				}

				if (y == 0) {
					Iy[pos] = getGradientY(pixels[pos], pixels[pos + width]);
				}
				else if (y == height - 1) {
					Iy[pos] = getGradientY(pixels[pos - width], pixels[pos]);
				}
				else {
					Iy[pos] = getGradientY(pixels[pos - width], pixels[pos + width]);
				}

			}
		}

		ofVec2f featureVector[size];

		float min_magnitude = numeric_limits<float>::max();
		float max_magnitude = 0;

		for (int i = 0; i < size; i++) {

			ofVec2f gf;

			//magnitude
			gf.x = sqrt(Ix[i] * Ix[i] + Iy[i] * Iy[i]);
			//oriantation
			gf.y = atan2(Iy[i], Ix[i]) * 180 / PI;
			gf.y = gf.y<0 ? gf.y + 360 : gf.y;
			featureVector[i] = gf;
		}


		float magnitudes[11 * 11];

		int featurePos = 0;
		for (int y = 0; y < appUtils::HOG_SIZE - 15; y += 8)
		{
			for (int x = 0; x<appUtils::HOG_SIZE - 15; x += 8)
			{
				float magnitudeSum = 0;
				float orientationSum = 0;

				float hist[9] = {0,0,0,0,0,0,0,0,0};

				for (int j = y; j<y + 16; j++)
				{
					for (int k = x; k<x + 16; k++)
					{
						int pos = j*appUtils::HOG_SIZE + k;

						if (featureVector[pos].y >= 0 && featureVector[pos].y <= 40) {
							hist[0] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>40 && featureVector[pos].y <= 80) {
							hist[1] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>80 && featureVector[pos].y <= 120) {
							hist[2] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>120 && featureVector[pos].y <= 160) {
							hist[3] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>160 && featureVector[pos].y <= 200) {
							hist[4] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>200 && featureVector[pos].y <= 240) {
							hist[5] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>240 && featureVector[pos].y <= 280) {
							hist[6] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>280 && featureVector[pos].y <= 320) {
							hist[7] += featureVector[pos].x;
						}
						else if (featureVector[pos].y>320 && featureVector[pos].y <= 360) {
							hist[8] += featureVector[pos].x;
						}
					}
				}

				int max = -1;
				float maxValue = 0;
				for (int i = 0; i < 9; i++) {

					if (hist[i]>maxValue) {
						maxValue = hist[i];
						max = i;
					}
				}
				features[featurePos] = max;
				magnitudes[featurePos] = maxValue;
				featurePos++;
			}
		}

		for (int i = 0; i < (11 * 11); i++) {
			min_magnitude = magnitudes[i]<min_magnitude ? magnitudes[i] : min_magnitude;
			max_magnitude = magnitudes[i]>max_magnitude ? magnitudes[i] : max_magnitude;
		}

		float threshold = min_magnitude + (max_magnitude - min_magnitude)*0.1f;

		for (int i = 0; i < (11 * 11); i++) {
			features[i] = magnitudes[i]>threshold ? features[i] : -1;
		}

		//cout << "Features processed" << endl;
	}

	static int getGradientX(int a, int c) {
		int grad = 0;

		grad = -a + c;

		return grad;
	}

	static int getGradientY(int a, int c) {
		int grad = 0;

		grad = a - c;

		return grad;
	}

	static float getEuclideanDist(std::array<float, 11 * 11> val1, std::array<float, 11 * 11> val2) {
		float dist = 0;
		for (int i = 0; i < (11 * 11); i++) {
			float buff = val1[i] - val2[i];
			dist += buff * buff;
		}
		return dist;
	}

	static void setFeatureImage(ofImage& image, std::array<float, 11 * 11> features) {

		const int w = 11;
		const int h = 11;

		ofPixels pix = ofPixels();

		pix.allocate(w, h, OF_IMAGE_COLOR);

		for (int i = 0; i< (11 * 11); i++) {

			int r = 0;
			int b = 0;
			int g = 0;

			cout << (int)features[i] << endl;

			switch ((int)features[i]) {
			case 0:
				r = 255;
				b = 0;
				g = 0;
				break;
			case 1:
				r = 255;
				b = 128;
				g = 0;
				break;
			case 2:
				r = 255;
				b = 255;
				g = 0;
				break;
			case 3:
				r = 0;
				b = 255;
				g = 0;
				break;
			case 4:
				r = 0;
				b = 255;
				g = 128;
				break;
			case 5:
				r = 0;
				b = 255;
				g = 255;
				break;
			case 6:
				r = 0;
				b = 0;
				g = 255;
				break;
			case 7:
				r = 128;
				b = 0;
				g = 255;
				break;
			case 8:
				r = 255;
				b = 0;
				g = 255;
				break;

			default:
				break;
			}


			pix.setColor(i, ofColor(r,g,b,255));
		}

		image.setFromPixels(pix);
	}

};