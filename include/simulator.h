#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__
#include "controller.h"
#include "vehicle.h"

typedef struct t_simulator{
    int n_vehicles;
    vehicle * vehicles;
    double current_time;
    double max_time;
    double time_increment;
    double vehicle_update_rate;
    int num_waypoints;
    int radius;
    double ** offset_waypoints;
    double turn_rate;
    void (*run)(struct t_simulator*);
    void (*runThreaded)(struct t_simulator*);
    control (*get_control)(struct t_simulator*,double);
} simulator;

typedef struct{
    vehicle* spec_vehicle;
    simulator* parent_sim;
} vehicle_plus;

simulator * create_simulator();
void run(struct t_simulator * sim);

#endif
