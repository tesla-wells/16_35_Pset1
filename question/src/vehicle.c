#include "../include/vehicle.h"

// create vehicle
vehicle * create_vehicle(double * starting_position, int num_waypoints, double ** offset_waypoints){
// standard vehicle methods
// Do another check of waypoints here to make sure they are in bounds and unique?
// I'm p sure with the optimal controller it is possible to tell if you can physically execute the maneuver. 
set_position(vehicle, starting_position);

vehicle->num_waypoints = num_waypoints;
vehicle->target_waypoints = offset_waypoints;

//There could be problem if waypoints are at more than a pi/4 angle to each other because our update state currently does "corner cutting" methodology of turning which allows for pi/4 per clock count of turn but gets increasingly less accurate as there is more corner :(. Clarify this later-- seems operational rather than coding oriented.

}

void set_position   (struct t_vehicle * v,double * values){
// x and y Must be in the range from [0,100)
// theta mus be -pi to pi

v->position = values;

}
void set_velocity   (struct t_vehicle * v,double * values){
//linear velocity range [5, 10] each (not total)
//angular velocity not exceed the range -pi/4 to pi/4
v->velocity = values;

}

void control_vehicle(struct t_vehicle * v);
{
	control.get_proportional_waypoint_control(v);
}

void update_state  (struct t_vehicle * v, double time){
	double* new_values = {v->position[0] + v->velocity[0]*time,
				v->position[1] + v->velocity[1]*time,
				v->position[2] + v->velocity[2]*time}

	set_position(v, new_values);
}

