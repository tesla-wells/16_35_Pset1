#ifndef __CLIENT_H__
#define __CLIENT_H__
#include "vehicle.h"
void open_server(char * ip, int port);
void send_data(char * data);
void send_vehicle_data(int n_vehicles, double * pos_x, double * pos_y, double * theta);
void send_vehicles(int n_vehicles, vehicle * vehicles);
void close_server();
// for starting and stopping the display server process.
void launch_display_server();
void close_display_server();
#endif