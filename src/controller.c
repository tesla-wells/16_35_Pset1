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

	double desiredAngle = atan2(opposite , adjacent);

	double angleDifference = desiredAngle - vehicle->position[2];

	if(angleDifference > M_PI){
		angleDifference = angleDifference - 2*M_PI;
	}
	if(angleDifference < -M_PI){
		angleDifference = angleDifference + 2*M_PI;
	}

	if(angleDifference > M_PI/4){
		angleDifference = M_PI/4 - 0.0001;
	}

	if(angleDifference <= -M_PI/4){
		angleDifference = -M_PI/4 + 0.0001;
	}

	vehicleController.angular_velocity = -1 * angleDifference;

	double distanceFromWaypoint = hypotenuse(vehicle->position, vehicle->current_waypoint);

	if(angleDifference < 0.25268){
		if(distanceFromWaypoint < 10){
			if(distanceFromWaypoint < 5.0){
				vehicleController.speed = 5.1;
			} else {
				vehicleController.speed = distanceFromWaypoint;
			}
		} else {
			vehicleController.speed = 8.0;
		}
	} else {
		vehicleController.speed = 5.1;
	}

	return vehicleController;
//return an angular velocity and linear velocity for the vehicle to execute that obeys the vehicles bounds
//
}

