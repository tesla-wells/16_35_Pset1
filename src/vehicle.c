#include "../include/vehicle.h"
#include "../include/controller.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


#define N_CONTROLLERS 1

//This value is normally in a utils file but is here for reasons explained in controller.c
double hypotenuse(double* pointA, double* pointB){
	float legA = pointA[0] - pointB[0];
	float legB = pointA[1] - pointB[1];
	float finalVal = sqrt(powf(legA, 2) + powf(legB, 2));
	return finalVal; 
}


//Validation functions go up here but would normally be separated out into their won validate.c file. See my github repo for proper structuring.
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


//Set up the controller struct here-- but ideally could go somewhere else?
control (*controllers[N_CONTROLLERS])(struct t_vehicle* v) = {
	&get_proportional_waypoint_control
};
int controller_idx = 0; 



void set_position   (struct t_vehicle * v,double * values){
	// x and y Must be in the range from [0,100)
	// theta mus be -pi to pi
	if(validatePosition(values) == 0){
		v->position[0] = values[0];
		v->position[1] = values[1];
		v->position[2] = values[2];
	}
}
void set_velocity   (struct t_vehicle * v,double * values){
	//linear velocity range [5, 10] each (not total)
	//angular velocity not exceed the range -pi/4 to pi/4
	if(validateVelocity(values) == 0){
		v->velocity[0] = values[0];
		v->velocity[1] = values[1];
		v->velocity[2] = values[2];
	}
}

void control_vehicle (struct t_vehicle * v){
	//if you have multiple control options this needs to be a switch with some other variable
	control controlRecommends = controllers[controller_idx](v);
	double equivValues[3] = {controlRecommends.speed * cos(v->position[2]),
			 controlRecommends.speed * sin(v->position[2]),
			 controlRecommends.angular_velocity};
	set_velocity(v, equivValues);

}

void update_state (struct t_vehicle * v, double time){
	double new_values[3] = {(v->position)[0] + (v->velocity)[0] * time,
				(v->position)[1] + (v->velocity)[1] * time,
				(v->position)[2] + (v->velocity)[2] * time};

	//printf("first velocity %f, %f, %f \n", new_values[0], new_values[1], new_values[2]);
	if(v->position[2] + v->velocity[2] * time > M_PI){
		new_values[2] = new_values[2] - 2*M_PI;
	}

	//printf("second velocity %f, %f, %f \n", new_values[0], new_values[1], new_values[2]);
	if(v->position[2] + v->velocity[2] * time < -M_PI){
		new_values[2] = new_values[2] + 2*M_PI;
	}
	
	//printf("third velocity %f, %f, %f \n", new_values[0], new_values[1], new_values[2]);
	if(validatePosition(new_values) == 0){
		set_position(v, new_values);
	}

	float hold = hypotenuse(v->position, v->current_waypoint);
	if( hold <= 5.0/2.0){
		v->current_waypoint_idx++;
		printf("%d new waypoint!", v->current_waypoint_idx);
		if(v->current_waypoint_idx >= v->num_waypoints){
			v->current_waypoint_idx = 0;
		}
		v->current_waypoint = v->target_waypoints[v->current_waypoint_idx];
	}	
	
	
}

// create vehicle
vehicle * create_vehicle(double * starting_position, int num_waypoints, double ** offset_waypoints){
	vehicle* initVehicle = malloc(sizeof(vehicle));
	set_position(initVehicle, starting_position);

	double** adjusted_points = malloc(num_waypoints*sizeof(double[2]));
	
	for(int i = 0; i < num_waypoints; i++){
		adjusted_points[i] = malloc(sizeof(double[2]));
		adjusted_points[i][0] = offset_waypoints[i][0] + starting_position[0];
		adjusted_points[i][1] = offset_waypoints[i][1] + starting_position[1];
	
	}

	if(validateWaypoints(num_waypoints, adjusted_points) == 0){
		initVehicle->num_waypoints = num_waypoints;
		initVehicle->target_waypoints = adjusted_points;
	}
	//Find the closest waypoint and assign that as your first stop
	float shortestDistance = INFINITY;

	//This isn't defined in the requirements but seems obvious. You might need to comment it out and assign the first one as the location
	for(int i = 0; i < num_waypoints; i++){
		double hold = hypotenuse(adjusted_points[i], starting_position);
		if (hold < shortestDistance){
			shortestDistance = hold;
			initVehicle->current_waypoint = adjusted_points[i];
			initVehicle->current_waypoint_idx = i;
		}
	}

	controller_idx = 0;	
	
	initVehicle->set_position = set_position;
	initVehicle->set_velocity = set_velocity;	
	initVehicle->control_vehicle = control_vehicle;
	initVehicle->update_state = update_state;

	control_vehicle(initVehicle);

	return initVehicle;

}
