

typedef long long int NumericalID;

//need to free the memory of the returned value (sorted=0:no sort, sorted=1:sort ascending, sorted=-1:sort descending w.r.t. the local longitudinal position inside the road)
NumericalID* (* get_all_ids)();


NumericalID (* get_all_ids_size)();


//need to free the memory of the returned value (sorted=0:no sort, sorted=1:sort ascending, sorted=-1:sort descending w.r.t. the local longitudinal position inside the road)
NumericalID* (* get_lane_free_ids)();
NumericalID (* get_lane_free_ids_size)();

char* (* get_vehicle_name)(NumericalID veh_id);


NumericalID* (* get_all_edges)();
NumericalID (* get_all_edges_size)();

NumericalID (* get_edge_of_vehicle)(NumericalID veh_id);

//need to free the memory of the returned value (sorted=0:no sort, sorted=1:sort ascending, sorted=-1:sort descending w.r.t. the local longitudinal position inside the road)
NumericalID* (* get_all_ids_in_edge)(NumericalID edge_id); 
NumericalID (* get_all_ids_in_edge_size)(NumericalID edge_id);

void (* apply_acceleration)(NumericalID veh_id, double accel_x, double accel_y);

double (* get_speed_x)(NumericalID veh_id);
double (* get_speed_y)(NumericalID veh_id);

double (* get_desired_speed)(NumericalID veh_id);
void (* set_desired_speed)(NumericalID veh_id, double new_des_speed);

double (* get_position_x)(NumericalID veh_id);
double (* get_position_y)(NumericalID veh_id);

double (* get_time_step_length)();
int (* get_current_time_step)();

NumericalID (* get_veh_type_id)(NumericalID veh_id);
char* (* get_veh_type_name)(NumericalID veh_id);



int (* get_seed)();


NumericalID* (* get_detectors_ids)();
NumericalID (* get_detectors_size)();
char* (* get_detector_name)(NumericalID detector_id);
int* (* get_detectors_values)();
int* (* get_density_per_segment_per_edge)(NumericalID edge_id, double segment_length);
int (* get_density_per_segment_per_edge_size)(NumericalID edge_id, double segment_length);

NumericalID (*insert_new_vehicle)(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y);

void simulation_initialize();
void simulation_step();
void simulation_finalize();

void event_vehicle_enter(NumericalID veh_id);

void event_vehicle_exit(NumericalID veh_id);

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2);
