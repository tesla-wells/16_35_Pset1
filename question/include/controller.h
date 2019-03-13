#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include "vehicle.h"
// defines the control class
typedef struct {
    double speed;
    double angular_velocity; // rad/s
} control;

// List a bunch of possible controller functions here that, given a vehicle, returns a control.
control get_proportional_waypoint_control(struct t_vehicle * vehicle);

#endif