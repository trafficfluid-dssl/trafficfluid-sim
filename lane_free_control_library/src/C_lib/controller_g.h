
void sim_initialize(void);


void sim_run(void);


void sim_finalize(void);


//Executed every time a new vehicle with id enter_id enters the road network
void sim_vehicle_enter(size_t enter_id);


//Executed every time a vehicle with id exit_id exits the road network
void sim_vehicle_exit(size_t exit_id);


//Executed every time two vehicles with ids crash_id_1, crash_id_2 collide
void sim_vehicles_collision(size_t crash_id_1, size_t crash_id_2);


//Executed every time a vehicle with id out_bound_id is located outside the road boundary
void sim_vehicle_out_of_bound(size_t out_bound_id);


//Executed once, before simulation begins. You may overwrite the initial placement of vehicles
void sim_initialize_vehicles(int n, int* type, double* x, double* y, double* vx);