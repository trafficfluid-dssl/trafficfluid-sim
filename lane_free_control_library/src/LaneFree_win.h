#include "libLaneFreePlugin_Export.h"
#include <stdarg.h>
typedef long long int NumericalID;

//need to free the memory of the returned value (sorted=0:no sort, sorted=1:sort ascending, sorted=-1:sort descending w.r.t. the local longitudinal position inside the road)
libLaneFreePlugin_EXPORT NumericalID* (* get_all_ids)();


libLaneFreePlugin_EXPORT NumericalID (* get_all_ids_size)();


//need to free the memory of the returned value (sorted=0:no sort, sorted=1:sort ascending, sorted=-1:sort descending w.r.t. the local longitudinal position inside the road)
libLaneFreePlugin_EXPORT NumericalID* (* get_lane_free_ids)();
libLaneFreePlugin_EXPORT NumericalID (* get_lane_free_ids_size)();

libLaneFreePlugin_EXPORT char* (* get_vehicle_name)(NumericalID veh_id);


libLaneFreePlugin_EXPORT NumericalID* (* get_all_edges)();
libLaneFreePlugin_EXPORT NumericalID (* get_all_edges_size)();

libLaneFreePlugin_EXPORT NumericalID (* get_edge_of_vehicle)(NumericalID veh_id);

//need to free the memory of the returned value (sorted=0:no sort, sorted=1:sort ascending, sorted=-1:sort descending w.r.t. the local longitudinal position inside the road)
libLaneFreePlugin_EXPORT NumericalID* (* get_all_ids_in_edge)(NumericalID edge_id);
libLaneFreePlugin_EXPORT NumericalID (* get_all_ids_in_edge_size)(NumericalID edge_id);

libLaneFreePlugin_EXPORT void (* apply_acceleration)(NumericalID veh_id, double accel_x, double accel_y);

libLaneFreePlugin_EXPORT double (* get_speed_x)(NumericalID veh_id);
libLaneFreePlugin_EXPORT double (* get_speed_y)(NumericalID veh_id);

libLaneFreePlugin_EXPORT double (* get_desired_speed)(NumericalID veh_id);
libLaneFreePlugin_EXPORT void (* set_desired_speed)(NumericalID veh_id, double new_des_speed);

libLaneFreePlugin_EXPORT double (* get_position_x)(NumericalID veh_id);
libLaneFreePlugin_EXPORT double (* get_position_y)(NumericalID veh_id);

libLaneFreePlugin_EXPORT double (* get_time_step_length)();
libLaneFreePlugin_EXPORT int (* get_current_time_step)();

libLaneFreePlugin_EXPORT NumericalID (* get_veh_type_id)(NumericalID veh_id);
libLaneFreePlugin_EXPORT char* (* get_veh_type_name)(NumericalID veh_id);



libLaneFreePlugin_EXPORT int (* get_seed)();


libLaneFreePlugin_EXPORT NumericalID* (* get_detectors_ids)();
libLaneFreePlugin_EXPORT NumericalID (* get_detectors_size)();
libLaneFreePlugin_EXPORT char* (* get_detector_name)(NumericalID detector_id);
libLaneFreePlugin_EXPORT int* (* get_detectors_values)();
libLaneFreePlugin_EXPORT int* (* get_density_per_segment_per_edge)(NumericalID edge_id, double segment_length);
libLaneFreePlugin_EXPORT int (* get_density_per_segment_per_edge_size)(NumericalID edge_id, double segment_length);

libLaneFreePlugin_EXPORT NumericalID (*insert_new_vehicle)(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y);

libLaneFreePlugin_EXPORT void (*print_to_sumo)(char* msg);

libLaneFreePlugin_EXPORT void simulation_initialize();
libLaneFreePlugin_EXPORT void simulation_step();
libLaneFreePlugin_EXPORT void simulation_finalize();

libLaneFreePlugin_EXPORT void event_vehicle_enter(NumericalID veh_id);

libLaneFreePlugin_EXPORT void event_vehicle_exit(NumericalID veh_id);

libLaneFreePlugin_EXPORT void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2);

void print_message(const char* format, ...) {
	va_list args;
	va_start(args, format);
	char buffer[256];
	vsprintf(buffer, format, args);
	print_to_sumo(buffer);
	va_end(args);
}