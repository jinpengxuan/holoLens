#pragma once

#include "ofxKinectForWindows2.h"
#include "ofMain.h"
#include "frame.h"
#include "applicationProperties.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = filesystem;

class imageUtils {

public:

	static void setCentroid(vector<ofVec3f>& positions, ofVec3f& centroid) {
		float positionSum = positions.size();
		
		for (ofVec3f& point : positions) {
			centroid.x += point.x;
			centroid.y += point.y;
			centroid.z += point.z;
		}
		centroid.x /= positionSum;
		centroid.y /= positionSum;
		centroid.z /= positionSum;
	}

	static float getAngleBetweenVectors(ofVec2f& v1, ofVec2f& v2) {
		float angle = atan2(v2.y, v2.x) - atan2(v1.y, v1.x);

		return angle;
	}

	static void setFingerMap(map<string, ofVec3f>& fingerMap, vector<ofVec3f>& clusters) {
		if (clusters.size() < 5)return;

		ofVec3f lastTriggerFinger = ofVec3f(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());
		if (fingerMap.size() == 5) {
			lastTriggerFinger = fingerMap["firstOuterFinger"];
		}

		//find thumb
		fingerMap.clear();
		int thumbIndex = -1;
		int clusterPos = 0;
		float largestDistance = 0;
		for (ofVec3f& pos : clusters) {
			int otherClusterPos = 0;
			float distance = 0;
			for (ofVec3f& otherPos : clusters) {
				if (clusterPos == otherClusterPos) {
					otherClusterPos++;
					continue;
				}
				float currentDistance = abs(otherPos.x - pos.x) + abs(otherPos.y - pos.y);
				distance += currentDistance;
				otherClusterPos++;
			}
			if (distance > largestDistance) {
				thumbIndex = clusterPos;
				largestDistance = distance;
			}
			clusterPos++;
		}
		if (thumbIndex >= 0)fingerMap["thumb"] = clusters.at(thumbIndex);
		
		//find other fingers
		int firstOuterFingerIndex = -1;
		int secondOuterFingerIndex = -1;
		int secondOuterFingerTempIndex = -1;
		clusterPos = 0;
		largestDistance = 0;
		for (ofVec3f& pos : clusters) {
			if (clusterPos == thumbIndex) {
				clusterPos++;
				continue;
			}
			int otherClusterPos = 0;
			float distance = 0;
			for (ofVec3f& otherPos : clusters) {
				if (otherClusterPos == clusterPos || otherClusterPos == thumbIndex) {
					otherClusterPos++;
					continue;
				}
				float currentDistance = abs(otherPos.x - pos.x) + abs(otherPos.y - pos.y);
				if (currentDistance > distance) {
					distance = currentDistance;
					secondOuterFingerTempIndex = otherClusterPos;
				}
				otherClusterPos++;
			}
			if (distance > largestDistance) {
				firstOuterFingerIndex = clusterPos;
				secondOuterFingerIndex = secondOuterFingerTempIndex;
			}
			clusterPos++;
		}
		if (firstOuterFingerIndex >= 0)fingerMap["firstOuterFinger"] = clusters.at(firstOuterFingerIndex);
		if (secondOuterFingerIndex >= 0)fingerMap["secondOuterFinger"] = clusters.at(secondOuterFingerIndex);

		//decide which one is trigger finger
		/*if (lastTriggerFinger.x != numeric_limits<float>::min()) {
			float firstFingerDistance = abs(fingerMap["firstOuterFinger"].x - lastTriggerFinger.x) + abs(fingerMap["firstOuterFinger"].y - lastTriggerFinger.y);
			float seccondFingerDistance = abs(fingerMap["secondOuterFinger"].x - lastTriggerFinger.x) + abs(fingerMap["secondOuterFinger"].y - lastTriggerFinger.y);
			if (seccondFingerDistance<firstFingerDistance) {
				ofVec3f temp = fingerMap["secondOuterFinger"];
				fingerMap["secondOuterFinger"] = fingerMap["firstOuterFinger"];
				fingerMap["firstOuterFinger"] = temp;
			}
		}*/
		
		int otherFingerIndex = 1;
		for (int i = 0; i < clusters.size(); i++) {
			if (i != thumbIndex && i != firstOuterFingerIndex && i != secondOuterFingerIndex) {
				string key = "otherFinger" + to_string(otherFingerIndex);
				fingerMap[key] = clusters.at(i);
				otherFingerIndex++;
			}
		}
	}

	static bool recursiveSetClusterIndex(int& clusterIndex, int x, int y, int width, int height, frame& clusterFrame, frame& clusterFrameIndex) {
		if (x < 1 || x >= width - 1 || y < 1 || y >= height - 1)return false;
		int index = x + y * width;

		if (clusterFrameIndex.pixels[index] > 0) return false;

		if (clusterFrame.pixels[index] == 1) {
			clusterFrameIndex.pixels[index] = clusterIndex;
			//left pixel
			recursiveSetClusterIndex(clusterIndex, x - 1, y, width, height, clusterFrame, clusterFrameIndex);
			//right pixel
			recursiveSetClusterIndex(clusterIndex, x + 1, y, width, height, clusterFrame, clusterFrameIndex);
			//bottom pixel									 
			recursiveSetClusterIndex(clusterIndex, x, y - 1, width, height, clusterFrame, clusterFrameIndex);
			//top pixel										 
			recursiveSetClusterIndex(clusterIndex, x, y + 1, width, height, clusterFrame, clusterFrameIndex);
			//bottom left pixel
			recursiveSetClusterIndex(clusterIndex, x - 1, y - 1, width, height, clusterFrame, clusterFrameIndex);
			//bottom right pixel
			recursiveSetClusterIndex(clusterIndex, x + 1, y - 1, width, height, clusterFrame, clusterFrameIndex);
			//top left pixel
			recursiveSetClusterIndex(clusterIndex, x - 1, y + 1, width, height, clusterFrame, clusterFrameIndex);
			//top right pixel
			recursiveSetClusterIndex(clusterIndex, x + 1, y + 1, width, height, clusterFrame, clusterFrameIndex);
			return true;
		}
		return false;
	}

	static void setPixelClusters(vector<ofVec3f>& coordinateClusers, frame& depthFrame) {
		int length = depthFrame.width*depthFrame.height;
		frame clusterFrame;
		frame clusterFrameIndex;
		clusterFrame.pixels = new int[length];
		clusterFrameIndex.pixels = new int[length];

		for (int i = 0; i < length; i++) {
			clusterFrame.pixels[i] = 0;
			clusterFrameIndex.pixels[i] = 0;
		}

		int frameCenterX = depthFrame.minXImg + depthFrame.widthImg / 2;
		int frameCenterY = depthFrame.minYImg + depthFrame.heightImg / 2;
		int radiusX = depthFrame.widthImg / 2;
		int radiusY = depthFrame.heightImg / 2;
		int radius = (radiusX + radiusY) / 2;

		for (int y = 0; y < depthFrame.height; y++) {
			for (int x = 0; x < depthFrame.width; x++) {
				int index = x + y * depthFrame.width;
				int distance = depthFrame.pixels[index];
				int distanceToCenter = abs(x - frameCenterX) + abs(y - frameCenterY);

				if (distance > depthFrame.nearPoint.z + 40 || distance < depthFrame.nearPoint.z || distanceToCenter < radius) {
					clusterFrame.pixels[index] = 0;
				}
				else {
					clusterFrame.pixels[index] = 1;
				}
			}
		}

		int clusterIndex = 1;
		for (int y = 1; y < depthFrame.height - 1; y++) {
			for (int x = 1; x < depthFrame.width - 1; x++) {
				int index = x + y * depthFrame.width;
				int markingTop = clusterFrame.pixels[index - depthFrame.width];
				int markingTopLeft = clusterFrame.pixels[index - depthFrame.width - 1];
				int markingTopRight = clusterFrame.pixels[index - depthFrame.width + 1];
				int markingLeft = clusterFrame.pixels[index - 1];
				int markingRight = clusterFrame.pixels[index + 1];
				int markingBottom = clusterFrame.pixels[index + depthFrame.width];
				int markingBottomLeft = clusterFrame.pixels[index + depthFrame.width - 1];
				int markingBottomRight = clusterFrame.pixels[index + depthFrame.width + 1];
				int sumMarking = markingTop + markingTopLeft + markingTopRight + markingLeft + markingRight + markingBottom + markingBottomLeft + markingBottomRight;

				if (sumMarking > 5) {
					bool newCluster = recursiveSetClusterIndex(clusterIndex, x, y, depthFrame.width, depthFrame.height, clusterFrame, clusterFrameIndex);
					if (newCluster)clusterIndex++;
				}
				else {
					clusterFrameIndex.pixels[index] = 0;
				}
			}
		}

		//ofstream myfile;
		//myfile.open("clusterFrame.txt");
		for (int y = 0; y < depthFrame.height; y++) {
			for (int x = 0; x < depthFrame.width; x++) {
				int index = x + y * depthFrame.width;
				int clusterIndex = clusterFrameIndex.pixels[index];
				int distance = depthFrame.pixels[index];

				//myfile << clusterIndex << " ";
				if (clusterIndex > 0 && clusterIndex < 6) {
					if (coordinateClusers.size() < clusterIndex) {
						coordinateClusers.push_back(ofVec3f(x, y, distance));
					}
					else if (coordinateClusers[clusterIndex - 1].z > distance) {
						coordinateClusers[clusterIndex - 1] = ofVec3f(x, y, distance);
					}
				}
			}
			//myfile << endl;
		}
		//myfile.close();

		delete[] clusterFrameIndex.pixels;
		delete[] clusterFrame.pixels;
	}

	static void setClusters(vector<ofVec3f>& depthCoords, vector<ofVec2f>& coordinateClusers, frame& frame, int clusterCount) {
		int clusterRadius = 20;
		for (ofVec3f& iteratorTemp : depthCoords) {
			float x = iteratorTemp.x;
			float y = iteratorTemp.y;
			float z = iteratorTemp.z;
			if (coordinateClusers.size() == clusterCount)break;
			if (z < (frame.nearPoint.z + 10)) {
				bool found = false;
				for (ofVec2f& iteratorCluster : coordinateClusers) {
					if (x >(iteratorCluster.x - clusterRadius)
						&& x <(iteratorCluster.x + clusterRadius)
						&& y >(iteratorCluster.y - clusterRadius)
						&& y < (iteratorCluster.y + clusterRadius)
						) {
						found = true;
					}
				}
				if (!found)coordinateClusers.push_back(ofVec2f(x, y));
			}
		}
	}

	static int getAccuracy(vector<std::array<float, 11 * 11>>& featuresReference, std::array<float, 11 * 11>& features) {
		int accuracy = numeric_limits<int>::max();
		for (std::array<float, 11 * 11> featuresRef : featuresReference) {

			int difference = imageUtils::getEuclideanDist(featuresRef, features);
			accuracy = difference < accuracy ? difference : accuracy;
		}

		accuracy = (2000.f - accuracy) / 2000.f * 100.f;

		accuracy = accuracy < 0 ? 0 : (accuracy > 100.f ? 100.f : accuracy);

		return accuracy;
	}

	static void setHandDepthImage(ofImage& handImage, frame& frame) {

		handImage.allocate(frame.widthImg, frame.heightImg, OF_IMAGE_GRAYSCALE);
		ofPixels pix = ofPixels();

		float depthAmount = (frame.maxZ - frame.nearPoint.z);

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
		handImage.resize(applicationProperties::HOG_SIZE, applicationProperties::HOG_SIZE);
	}

	static void setHandColorImage(ofxKFW2::Device& kinect, ofImage& handImage, frame& frame, const ofShortPixels& depthPixels, ofPixels& colorPixels) {

		//mapping coordinates from depth to color image and vice versa
		ICoordinateMapper * coordinateMapper;

		vector<ofVec2f> colorCoords;
		colorCoords.resize(applicationProperties::DEPTH_SIZE);

		HRESULT hresult = kinect.getSensor()->get_CoordinateMapper(&coordinateMapper);

		if (FAILED(hresult)) {
			ofLog() << "CoordinateMapper Not Found";
		}

		coordinateMapper->MapDepthFrameToColorSpace(applicationProperties::DEPTH_SIZE, (UINT16 *)depthPixels.getPixels(), applicationProperties::DEPTH_SIZE,(ColorSpacePoint *)colorCoords.data());

		int minX = frame.minX + frame.minXImg;
		int minY = frame.minY + frame.minYImg;

		int maxX = frame.minX + frame.maxXImg;
		int maxY = frame.minY + frame.maxYImg;

		handImage.allocate(frame.widthImg, frame.heightImg, OF_IMAGE_COLOR);
		ofPixels pix = ofPixels();

		pix.allocate(frame.widthImg, frame.heightImg, OF_IMAGE_COLOR);
		pix.setColor(0);

		for (int y = minY; y < maxY; y++) {
			for (int x = minX; x < maxX; x++) {
				int index = x + y*depthPixels.getWidth();
				ofVec2f colorPosition = colorCoords[index];
				int colorIndex = colorPosition.x + colorPosition.y*colorPixels.getWidth();

				int val = colorPixels.getBitsPerChannel();
				char c = colorPixels[colorIndex];

				ofColor colorPix = colorPixels.getColor(colorIndex);

				int thumbIndex = (x - minX) + (y - minY) * frame.widthImg;
				pix.setColor(thumbIndex, colorPix);
			}
		}

		handImage.setFromPixels(pix);
		handImage.resize(applicationProperties::HOG_SIZE, applicationProperties::HOG_SIZE);
	}

	static void setDepthCoordinates(frame& depthFrame) {
		int minX = numeric_limits<int>::max();
		int minY = numeric_limits<int>::max();

		int maxX = numeric_limits<int>::min();
		int maxY = numeric_limits<int>::min();

		for (int y = 0; y < depthFrame.height; y++) {
			for (int x = 0; x < depthFrame.width; x++) {
				int index = x + y*depthFrame.width;
				int distance = depthFrame.pixels[index];

				if (distance > depthFrame.maxZ || distance < depthFrame.nearPoint.z) {
					continue;
				}

				minX = x < minX ? x : minX;
				maxX = x > maxX ? x : maxX;
				minY = y < minY ? y : minY;
				maxY = y > maxY ? y : maxY;
			}
		}

		depthFrame.minXImg = minX;
		depthFrame.maxXImg = maxX;
		depthFrame.minYImg = minY;
		depthFrame.maxYImg = maxY;
		depthFrame.widthImg = maxX - minX;
		depthFrame.heightImg = maxY - minY;
	}

	static void setFrame(frame& frameObj, const ofShortPixels& pixels) {
		frameObj.nearPoint.z = numeric_limits<int>::max();
		int length = frameObj.width*frameObj.height;
		frameObj.pixels = new int[length];

		for (int i = 0; i < length; i++) {
			frameObj.pixels[i] = numeric_limits<int>::max();
		}

		for (int y = frameObj.minY; y < frameObj.maxY; y ++) {
			for (int x = frameObj.minX; x < frameObj.maxX; x ++) {
				int index = x + y*pixels.getWidth();
				int indexFrame = (x - frameObj.minX) + (y - frameObj.minY)*frameObj.width;
				int distance = pixels[index];
				if (distance < 500) {
					frameObj.pixels[indexFrame] = 800;
					continue;
				} 
				else if (distance > 800) {
					frameObj.pixels[indexFrame] = 800;
					continue;
				}

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
					if (distance < frameObj.nearPoint.z) {
						frameObj.nearPoint.x = x;
						frameObj.nearPoint.y = y;
						frameObj.nearPoint.z = distance;

					}
				}
			}
		}

		frameObj.maxZ = frameObj.nearPoint.z + 80.f;
	}

	static void setFeatureVector(const ofPixels &pixels, std::array<float, 11 * 11> &features) {

		const int width = applicationProperties::HOG_SIZE;
		const int height = applicationProperties::HOG_SIZE;
		const int size = applicationProperties::HOG_SIZE * applicationProperties::HOG_SIZE;
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
		for (int y = 0; y < applicationProperties::HOG_SIZE - 15; y += 8)
		{
			for (int x = 0; x<applicationProperties::HOG_SIZE - 15; x += 8)
			{
				float magnitudeSum = 0;
				float orientationSum = 0;

				float hist[9] = {0,0,0,0,0,0,0,0,0};

				for (int j = y; j<y + 16; j++)
				{
					for (int k = x; k<x + 16; k++)
					{
						int pos = j*applicationProperties::HOG_SIZE + k;

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