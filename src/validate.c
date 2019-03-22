#include "../include/controller.h"
#include "../include/vehicle.h"
#include "../include/utils.h"
#include "../include/validate.h"
#include <math.h>
#include <stdio.h>

int validateInBounds(double x, double y){
	int error = 0;
	//validate that x and y are [0, 100)
	if(x < 0 || x >= 100 || y < 0 || y >= 100){
		error = 1;
		printf("%f, %f is out of bounds \n", x, y);
	}
	return error;
}

int validateWaypoints(int num_waypoints, double** offset_waypoints){
	int error = 0;
	//Validate that there are no duplicate waypoints
	for(int i=0; i < num_waypoints - 1; i++){
		for(int j=i+1; j < num_waypoints; j++){
			if (offset_waypoints[i] == offset_waypoints[j]){
				error = 1;
				printf("You have a duplicate waypoint \n");
			}
		}
	}
	//validate that all waypoints are within bounds
	for(int i=0; i < num_waypoints; i++){
		error = (error ||validateInBounds(offset_waypoints[i][0], offset_waypoints[i][1]));
	}
	
	
	//Bonus: Validate if the program given the constraints can attain each waypoint
	return error;
}

int validatePosition(double* values){
	int error = 0;
	
	error = validateInBounds(values[0], values[1]); 
	//validate that theta is [-pi, pi)
	if(values[2] < -1 * M_PI || values[2] >= M_PI){
		error = 1;
		printf("Your angle position is out of bounds \n");
	}
	return error;
}

int validateVelocity(double* values){
	int error = 0;
	//validate that x and y together are < 10
	double pointZero[3] = {0.0, 0.0, 0.0};
	float velocity = hypotenuse(values, pointZero);
	if(velocity < 5 || velocity > 10){
		printf("your linear velocity is out of bounds \n");
		error = 1;
	}
	//validate that the theta velocity is < -pi/4 to pi/4
	if(values[2] < M_PI / -4 || values[2] > M_PI / 4){
		printf("You are turnin too fast \n");
		error = 1;
	}
	return error;
}
