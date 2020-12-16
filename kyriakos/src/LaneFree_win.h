#ifdef DEFINE_VARIABLES
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* DEFINE_VARIABLES */
#include "libLaneFreePlugin_EXPORT.h"
#include <stdarg.h>


typedef long long int NumericalID;


//Note: memory management is performed automatically. You do not free the pointers you receive through this API (See LaneFree.cpp for usage)

//returns all ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID* (*get_all_ids)();
//returns the size of all ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_all_ids_size)();


//returns all lane free ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_lane_free_ids)();
//returns the size of all lane free ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_lane_free_ids_size)();


//returns the vehicle's name based on the id argument
EXTERN libLaneFreePlugin_EXPORT char* (* get_vehicle_name)(NumericalID veh_id);


//returns an array with all the edges
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_edges)();


//returns the size of all the edges
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_all_edges_size)();


//returns the id of the edge for a given vehicle, based on the vehicle's id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_edge_of_vehicle)(NumericalID veh_id);


//returns all ids in a given edge, based on the edge's id, and ordered according to their longitudinal position
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_ids_in_edge)(NumericalID edge_id);
//returns the size of all ids in a given edge, based on the edge's id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_all_ids_in_edge_size)(NumericalID edge_id);


//apply longitudinal acceleration accel_x m/s^2 and lateral acceleration accel_y m/s^2 to the vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT void (* apply_acceleration)(NumericalID veh_id, double accel_x, double accel_y);


//returns the longitudinal speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT double (* get_speed_x)(NumericalID veh_id);


//returns the longitudinal speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT double (* get_speed_y)(NumericalID veh_id);


//returns the desired speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT double (* get_desired_speed)(NumericalID veh_id);


//sets the desired speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT void (* set_desired_speed)(NumericalID veh_id, double new_des_speed);


//returns the longitudinal position of a vehicle based on the local coordinates (according to the road the vehicle is now)
EXTERN libLaneFreePlugin_EXPORT double (* get_position_x)(NumericalID veh_id);


//returns the lateral position of a vehicle based on the local coordinates (according to the road the vehicle is now)
EXTERN libLaneFreePlugin_EXPORT double (* get_position_y)(NumericalID veh_id);


//returns the relative longitudinal position of a vehicle with respect to an ego vehicle
EXTERN libLaneFreePlugin_EXPORT double (*get_relative_position_x)(NumericalID ego_id, NumericalID other_id);


//returns the relative lateral position of a vehicle with respect to an ego vehicle
EXTERN libLaneFreePlugin_EXPORT double (*get_relative_position_y)(NumericalID ego_id, NumericalID other_id);


//determines if a given vehicle will be transported at the beginning of the road edge it currently is into, based on the value of circular (true:enables the behavior, false:disables the behavior)
EXTERN libLaneFreePlugin_EXPORT void (*set_circular_movement)(NumericalID veh_id, bool circular);

//returns the time-step length
EXTERN libLaneFreePlugin_EXPORT double (* get_time_step_length)();


//returns the current time-step
EXTERN libLaneFreePlugin_EXPORT int (* get_current_time_step)();


//returns the type of a given vehicle id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_veh_type_id)(NumericalID veh_id);


//returns the name of a given type id
EXTERN libLaneFreePlugin_EXPORT char* (* get_veh_type_name)(NumericalID veh_id);


//returns the length of a given vehicle id
EXTERN libLaneFreePlugin_EXPORT double (*get_veh_length)(NumericalID veh_id);

//returns the width of a given vehicle id
EXTERN libLaneFreePlugin_EXPORT double (*get_veh_width)(NumericalID veh_id);

//returns the length of a given edge id
EXTERN libLaneFreePlugin_EXPORT double (*get_edge_length)(NumericalID edge_id);

//returns the width of a given edge id
EXTERN libLaneFreePlugin_EXPORT double (*get_edge_width)(NumericalID edge_id);


//returns the seed
EXTERN libLaneFreePlugin_EXPORT int (* get_seed)();


//returns the ids of the detectors
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_detectors_ids)();


//returns the size of the ids of the detectors
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_detectors_size)();


//returns the detector's name, based on the detector id
EXTERN libLaneFreePlugin_EXPORT char* (*get_detector_name)(NumericalID detector_id);


//returns the values of all detectors (number of vehicles for each detector)
EXTERN libLaneFreePlugin_EXPORT int* (*get_detectors_values)();


//returns the density of vehicles per segment for a given edge id, and a segment length
EXTERN libLaneFreePlugin_EXPORT int* (*get_density_per_segment_per_edge)(NumericalID edge_id, double segment_length);
//returns the size of the array provided above
EXTERN libLaneFreePlugin_EXPORT int (*get_density_per_segment_per_edge_size)(NumericalID edge_id, double segment_length);


//insert a new vehicle (route_id and type_id need to be defined in the scenario tested)
EXTERN libLaneFreePlugin_EXPORT NumericalID (*insert_new_vehicle)(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y);



//is called once before the first time-step
EXTERN libLaneFreePlugin_EXPORT void simulation_initialize();

//is called in every time-step
EXTERN libLaneFreePlugin_EXPORT void simulation_step();

//is called when the simulation ends
EXTERN libLaneFreePlugin_EXPORT void simulation_finalize();


//is called when a new vehicle enters
EXTERN libLaneFreePlugin_EXPORT void event_vehicle_enter(NumericalID veh_id);

//is called when a new vehicle exits
EXTERN libLaneFreePlugin_EXPORT void event_vehicle_exit(NumericalID veh_id);


//is called when two vehicles collide
EXTERN libLaneFreePlugin_EXPORT void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2);

