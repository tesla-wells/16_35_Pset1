#include "vehicle.h"
#include "controller.h"
#include <math.h>
#include <stdio.h>

// List a bunch of possible controller functions here that, given a vehicle, returns a control.
control get_proportional_waypoint_control(struct t_vehicle * vehicle){

control vehicleController;

vehicleController.speed = 9.0;
vehicleController.angular_velocity = 1;
//return an angular velocity and linear velocity for the vehicle to execute that obeys the vehicles bounds
//
}

control get_circumcenter_control(struct t_vehicle * vehicle){

}
