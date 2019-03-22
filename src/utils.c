#include <math.h>
#include "../include/utils.h"

double hypotenuse(double* pointA, double* pointB){
	float legA = pointA[0] - pointB[0];
	float legB = pointA[1] - pointB[1];
	float finalVal = sqrt(powf(legA, 2) + powf(legB, 2));
	return finalVal; 
}
