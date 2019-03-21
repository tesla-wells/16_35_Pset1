#include "../include/vehicle.h"
#include "../include/controller.h"
#include "validate.h"
#include "utils.h"
#include <math.h>

#define N_CONTROLLERS 1

control (*controllers[N_CONTROLLERS])(struct t_vehicle* v) = {
	&get_proportional_waypoint_control
};

int controller_idx = 0; 

// create vehicle
vehicle * create_vehicle(double * starting_position, int num_waypoints, double ** offset_waypoints){
	vehicle* initVehicle;

	set_position(initVehicle, starting_position);

	if(validateWaypoints(num_waypoints, offset_waypoints) == 0){
		initVehicle->num_waypoints = num_waypoints;
		initVehicle->target_waypoints = offset_waypoints;
	}
	//Find the closest waypoint and assign that as your first stop
	float shortestDistance = INFINITY;

	//This isn't defined in the requirements but seems obvious. You might need to comment it out and assign the first one as the location
	for(int i = 0; i < num_waypoints; i++){
		float hold = hypotenuse(offset_waypoints[i], starting_position);
		if (hold < shortestDistance){
			shortestDistance = hold;
			initVehicle->current_waypoint = offset_waypoints[i];
			initVehicle->current_waypoint_idx = i;
		}
	}

	controller_idx = 0;	

	return initVehicle;

}

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
	control waypoint = controllers[controller_idx](v);
}

void update_state (struct t_vehicle * v, double time){
	double new_values = {v->position[0] + v->velocity[0]*time,
				v->position[1] + v->velocity[1]*time,
				v->position[2] + v->velocity[2]*time};
	
	if(validatePosition(&new_values) == 0){
		set_position(v, &new_values);
		if(hypotenuse(v->position, v->current_waypoint) <= 5.0/2.0){
			v->current_waypoint_idx++;
			if(v->current_waypoint_idx >= v->num_waypoints){
				v->current_waypoint_idx = 0;
			}
			v->current_waypoint = v->target_waypoints[v->current_waypoint_idx];
		}	
	}
	
}
