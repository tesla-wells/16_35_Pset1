#include "simulator.h"
#include "vehicle.h"
#include <stdlib.h>
#include "client.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
void run(struct t_simulator * sim){
    open_server(IP,PORTNUM);

	clock_t start, stop;
    float numberOfIterations = 0.0;
	start = clock();	
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
            numberOfIterations++;
	    v->control_vehicle(v);
            v->update_state(v,sim->time_increment); // delta t
        }
        usleep(sim->time_increment*1e6); // sleep for roughly the time increment so we get quasi-realtime behavior
    }
	stop = clock();
    float totalSeconds = stop - start; 
    printf("\n The full runtime was %f milliseconds, and the average runtime was %f milliseconds", totalSeconds, totalSeconds/numberOfIterations); 
   close_server();
}

void* vehicle_thread(void* args){
	vehicle_plus* inputs;
	inputs = (vehicle_plus*) args;
	inputs->spec_vehicle->control_vehicle(inputs->spec_vehicle);
	inputs->spec_vehicle->update_state(inputs->spec_vehicle, inputs->parent_sim->time_increment);
	return 0;
}

void runThreaded(struct t_simulator * sim){
    open_server(IP,PORTNUM);

    clock_t start, stop;
    float numberOfIterations = 0.0;
	start = clock();	
    sim->current_time = 0.0;
    double time_vehicle_message = 0.0;

    vehicle_plus* vehicle_thread_inputs = malloc(sizeof(vehicle_plus)*sim->n_vehicles);

    for(int i =0; i < sim->n_vehicles; i++){
	vehicle_thread_inputs[i].spec_vehicle = &(sim->vehicles[i]);
	vehicle_thread_inputs[i].parent_sim = sim;
    }

    while (sim->current_time < sim->max_time) {

        printf("\rt = %f",sim->current_time);
	pthread_t* thread_list = malloc(sizeof(pthread_t)*sim->n_vehicles);
	for(int i = 0; i < sim->n_vehicles; i++){
		pthread_create(&thread_list[i], NULL, vehicle_thread, (void*) &vehicle_thread_inputs[i]); 
	}

	for(int i=0; i < sim->n_vehicles; i++){
		pthread_join(thread_list[i], NULL);
	}

	numberOfIterations++;
        sim->current_time += sim->time_increment;
        usleep(sim->time_increment*1e6); // sleep for roughly the time increment so we get quasi-realtime behavior
    }
	stop = clock();
    float totalSeconds = stop - start; 
    printf("\n The full runtime was %f milliseconds, and the average runtime was %f milliseconds.", totalSeconds, totalSeconds/numberOfIterations); 
   close_server();
}

simulator * create_simulator(){
    // initialization
    simulator * sim = malloc(sizeof(simulator));
    sim->run = &run;
    sim->runThreaded = &runThreaded;
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
    sim->max_time = 10.0;
    sim->current_time = 0.0;
    sim->time_increment = 0.01;
    return sim;
}

