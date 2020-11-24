#pragma once

#include <math.h>
#include <float.h>

using namespace std;

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp = false) {

	if (fabs(inputMin - inputMax) < FLT_EPSILON) {
		return outputMin;
	}
	else {
		float outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);

		if (clamp) {
			if (outputMax < outputMin) {
				if (outVal < outputMax)outVal = outputMax;
				else if (outVal > outputMin)outVal = outputMin;
			}
			else {
				if (outVal > outputMax)outVal = outputMax;
				else if (outVal < outputMin)outVal = outputMin;
			}
		}
		return outVal;
	}

}