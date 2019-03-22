#include "../include/vehicle.h"
#include "../include/controller.h"
#include "../include/utils.h"
#include <math.h>
#include <stdio.h>

// List a bunch of possible controller functions here that, given a vehicle, returns a control.
control get_proportional_waypoint_control(struct t_vehicle * vehicle){

	control vehicleController;

	double opposite = vehicle->position[1] - vehicle->current_waypoint[1];
	double adjacent = vehicle->position[0] - vehicle->current_waypoint[0];

	double desiredAngle = atan(opposite/adjacent);
	if(adjacent < 0){
		if(opposite < 0){
			desiredAngle = desiredAngle - M_PI;
		} else {
			desiredAngle = desiredAngle + M_PI;
		}
	}

	double angleDifference = desiredAngle - vehicle->position[2];

	if(angleDifference > M_PI){
		angleDifference = -2*M_PI + M_PI;
	}

	if(angleDifference >= M_PI/4){
		angleDifference = M_PI/4 - 0.0001;
	}

	if(angleDifference < -M_PI/4){
		angleDifference = -M_PI/4;
	}

	vehicleController.angular_velocity = angleDifference;

	double distanceFromWaypoint = hypotenuse(vehicle->position, vehicle->current_waypoint);

	if(angleDifference < 0.25268){
		if(distanceFromWaypoint < 10){
			vehicleController.speed = distanceFromWaypoint;
		} else {
			vehicleController.speed = 10.0;
		}
	} else {
		vehicleController.speed = 5.0;
	}
	
	return vehicleController;
//return an angular velocity and linear velocity for the vehicle to execute that obeys the vehicles bounds
//
}

