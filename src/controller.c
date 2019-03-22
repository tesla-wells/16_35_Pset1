#include "../include/vehicle.h"
#include "../include/controller.h"
#include <math.h>
#include <stdio.h>


//In my REAL code, I have this hypotenuse piece in a separate utils file where it can be accessed by all of my controllers. For simplicity of consolidation, I copied it both here and in the vehicle.c file and renamed them. I don't think this method should go here but for the sake of the grader it needs to be here. Please see my github to see the actual placement of this method.
double hypo(double* pointA, double* pointB){
	float legA = pointA[0] - pointB[0];
	float legB = pointA[1] - pointB[1];
	float finalVal = sqrt(powf(legA, 2) + powf(legB, 2));
	return finalVal; 
}

//required controller
control get_proportional_waypoint_control(struct t_vehicle * vehicle){

	control vehicleController;
	
	//calculated opposite and adjacnet, then uses atan2 to find the angle
	double opposite = vehicle->position[1] - vehicle->current_waypoint[1];
	double adjacent = vehicle->position[0] - vehicle->current_waypoint[0];

	double desiredAngle = atan2(opposite , adjacent);

	//calculates PROPORTIONAL difference
	double angleDifference =  desiredAngle - vehicle->position[2];
	//Puts it in the correct period 
	if(angleDifference > M_PI){
		angleDifference = angleDifference - 2*M_PI;
	}
	if(angleDifference < -M_PI){
		angleDifference = angleDifference + 2*M_PI;
	}

	//caps the angle differences
	if(angleDifference > M_PI/4){
		angleDifference = M_PI/4 - 0.0001;
	}

	if(angleDifference <= -M_PI/4){
		angleDifference = -M_PI/4 + 0.0001;
	}

	vehicleController.angular_velocity = -1 * angleDifference;


	//slows down and speeds up with some idea of how much it needs to turn and otherwise caps it at the min or max based on how close you are.
	double distanceFromWaypoint = hypo(vehicle->position, vehicle->current_waypoint);

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

	//printf("%f speed wanted, %f angle wanted \n", vehicleController.speed, vehicleController.angular_velocity);
	
	return vehicleController;
//return an angular velocity and linear velocity for the vehicle to execute that obeys the vehicles bounds
//
}

