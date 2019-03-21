while (sim->current_time < sim->max_time) {
    // other code
    // ...
    for (vehicle * v = sim->vehicles; v < sim->vehicles + sim->n_vehicles; v++){
        v->control_vehicle(v);
        v->update_state(v,sim->time_increment);
    }
    usleep(sim->time_increment*1e6);
}
