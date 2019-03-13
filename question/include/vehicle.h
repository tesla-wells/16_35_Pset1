#ifndef __VEHICLE_H__
#define __VEHICLE_H__
typedef struct t_vehicle {
    double position [3]; //x,y,theta
    double velocity [3]; //dx,dy,dtheta
    double ** target_waypoints;  // List of x, y points to drive to.
    double * current_waypoint;  // The current x, y point you're aiming for.
    int num_waypoints;
    int current_waypoint_idx;
    // vehicle function calls
    void (*set_position)   (struct t_vehicle * v, double * values); // applies constraints and updates values.
    void (*set_velocity)   (struct t_vehicle * v, double * values); // applies constraints and updates values.
    void (*control_vehicle)(struct t_vehicle * v);  // calculates and applies controls to vehicle
    void (*update_state)   (struct t_vehicle * v, double time); // updates vehicle states based on dynamics
} vehicle;

// create vehicle
vehicle * create_vehicle(double * starting_position, int num_waypoints, double ** offset_waypoints);
// standard vehicle methods
void set_position   (struct t_vehicle * v,double * values);
void set_velocity   (struct t_vehicle * v,double * values);
void control_vehicle(struct t_vehicle * v);
void update_state   (struct t_vehicle * v, double time);

#endif
