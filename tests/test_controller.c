#include <string.h>
#include <stlib.h>
#include <math.h>
#include "vehicle.h"
#include "controller.h"
#include "utils.h"
#include "validate.h"

int main (int argc, char ** argv) {
	if(strcmp(argv[1], "control1"){
		//test that origin to four points left returns the same thing as displaced from origin
		double position[3] = {0, 0, 0}
		double** offset_waypoints = malloc(2 * sizeof(double));
		offset_waypoints[0] = {4, 0};

		vehicle* v = create_vehicle(position, 1, offset_waypoints);
		v->control_vehicle(v);		

		double position1[3] = {0, 0, 0}
		double** offset_waypoints1 = malloc(2 * sizeof(double));
		offset_waypoints1[0] = {4, 0};

		vehicle* v1 = create_vehicle1(position1, 2, offset_waypoints1);
		v1->control_vehicle(v1);

		if(v->velocity[0] == v1->velocity[0] &&	v->velocity[1] == v1->velocity[1] && v->velocity[2] == v1->velocity[2]){
			return 0;
		}  else {
			return -1;
		}
	} else if(strcmp(argv[1], "control2")){
		//test that you turn no more than PI/4
		double position[3] = {0, 0, 0}
		double** offset_waypoints = malloc(2 * sizeof(double));
		offset_waypoints[0] = {-1, 4};

		vehicle* v = create_vehicle(position, 1, offset_waypoints);
		v->control_vehicle(v);

		if(v->velocity[2] < M_PI_4){
			return 0;
		} else {
			return -1;
		}
	} else if(strcmp(argv[1], "control3")){
		//test that you turn in the correct direction
		double position[3] = {0, 0, 0}
		double** offset_waypoints = malloc(2 * sizeof(double));
		offset_waypoints[0] = {-1, -4};

		vehicle* v = create_vehicle(position, 1, offset_waypoints);
		v->control_vehicle(v);

		if(v->velocity[2] < 0 && v -> velocity[2] >= -M_PI/4 ){
			return 0;
		} else {
			return -1;
		}
	} else if(strcmp(argv[1], "update1")){
		//Cross over from pi to -pi
		//UNFINISHED
		double position[3] = {0, 0, M_PI*4/5}
		return 0;	
		
	} else if(strcmp(argv[1], "update2")){
		//Try to launch yourself off the screen
		//UNFINISHED
		return 0;
	} else if(strcmp(argv[2], "udpate3")){
		
		//UNFINISHED
		//Try to update the velocity incorrectly
		return 0;
	}
}

