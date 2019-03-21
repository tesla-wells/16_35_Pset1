#include "simulator.h"
#include "vehicle.h"
#include <stdlib.h>
#include "client.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>
simulator * create_simulator(){
    // initialization
    simulator * sim = malloc(sizeof(simulator));
    sim->run = &run;
    sim->n_vehicles = 0;
    // update rate for display server
    sim->vehicle_update_rate = 25; // hz
    // Create "offset waypoints", a series of waypoints for the vehicles to follow.
    // Note that these waypoints are for the *relative* waypoints relative to the
    // starting point of the vehicle.
    sim->num_waypoints = 5;
    sim->radius = 30;
    sim->offset_waypoints = malloc((sim->num_waypoints + 1) * 2 * sizeof(double));
    for (int i = 0; i < sim->num_waypoints + 1; i++) {
        sim->offset_waypoints[i] = malloc(2 * sizeof(double));
        sim->offset_waypoints[i][0] = sim->radius * cos( i * 2 * M_PI / (sim->num_waypoints));
        sim->offset_waypoints[i][1] = sim->radius * sin( i * 2 * M_PI / (sim->num_waypoints));
    }
    // simulator settings
    sim->max_time = 100.0;
    sim->current_time = 0.0;
    sim->time_increment = 0.01;
    return sim;
}
void run(struct t_simulator * sim){
    open_server(IP,PORTNUM);
    sim->current_time = 0.0;
    double time_vehicle_message = 0.0;
    while (sim->current_time < sim->max_time) {
        printf("\rt = %f",sim->current_time);
        time_vehicle_message += sim->time_increment;
        sim->current_time += sim->time_increment;
        if (time_vehicle_message > 1.0/sim->vehicle_update_rate) {
            send_vehicles(sim->n_vehicles,sim->vehicles);
            time_vehicle_message = 0.0;
        }
        for (vehicle * v = sim->vehicles; v < sim->vehicles + sim->n_vehicles; v++){
            v->control_vehicle(v);
            v->update_state(v,sim->time_increment); // delta t
        }
        usleep(sim->time_increment*1e6); // sleep for roughly the time increment so we get quasi-realtime behavior
    }
    close_server();
}
