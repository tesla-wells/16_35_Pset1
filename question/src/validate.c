#include "../include/controller.h"
#include "../include/vehicle.h"
#include "utils.h"
#include "validate.h"
#include <math.h>

int validateWaypoints(int num_waypoints, double** offset_waypoints){
	//Validate that there are no duplicate waypoints
	//validate that all waypoints are within bounds
	//Bonus: Validate if the program given the constraints can attain each waypoint
	return 0;
}

int validatePosition(double* values){
	//validate that x and y are [0, 100)
	//validate that theta is [-pi, pi)
	//remember pi is M_PI
	return 0;
}

int validateVelocity(double* values){
	//validate that x and y together are < 10
	//validate that the theta velocity is < -pi/4 to pi/4
	return 0;
}
