#pragma once

class gestureTracker {

public:
	
	void startDrag();
	void stopDrag();

	bool dragged = false;
	int trackingTime = 0;

	float rotationDegree = 0.f;
	int translation = 0;

};
