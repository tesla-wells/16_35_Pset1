#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <client.h>
#include <unistd.h>
#include "simulator.h"
#include "vehicle.h"

int main(int argc, char *argv[])
{
    // this launches the display server (in a separate process)
    launch_display_server();
    // add a pause for python to start up
    // if you get error messages about sockets,
    // a good start would be to increase this time.
    sleep(1);
    // create the simulator
    simulator * sim = create_simulator();
    printf("quit the program - 'q'\n");
    // create 3 vehicles and add them to the simulator
    sim->n_vehicles = 3;
    sim->vehicles = malloc(sim->n_vehicles * sizeof(vehicle));
    for (int i = 0; i < sim->n_vehicles; i++) {
        double pos [3] = {i*10+30,i*10+30,0};
        vehicle * vehicle = create_vehicle(pos, sim->num_waypoints, sim->offset_waypoints);
        // creates a copy so we can free the original vehicle after
        sim->vehicles[i] = *vehicle;
        free(vehicle);
    }
    sim->run(sim);
    printf("press any key to kill the program\n");
    getchar();
    close_display_server();
}
