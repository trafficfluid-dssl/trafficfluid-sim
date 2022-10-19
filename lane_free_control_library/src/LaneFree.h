#ifdef DEFINE_VARIABLES
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* DEFINE_VARIABLES */

#ifdef __unix__
#define libLaneFreePlugin_EXPORT /* nothing */
#elif defined(WIN32)
#include "libLaneFreePlugin_EXPORT.h"
#endif


typedef long long int NumericalID;


// Note: memory management for all returned pointers is performed internally. You should not free the pointers received through this API (See LaneFree.cpp for usage example)

// returns all ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_ids)();
// returns the size of all ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_all_ids_size)();


// returns all lane free ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_lane_free_ids)();
// returns the size of all lane free ids in the network
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_lane_free_ids_size)();


// returns the vehicle's name based on the id argument
EXTERN libLaneFreePlugin_EXPORT char* (* get_vehicle_name)(NumericalID veh_id);


// returns an array with all the edges
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_edges)();


// returns the size of all the edges
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_all_edges_size)();


// returns the id of the edge for a given vehicle, based on the vehicle's id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_edge_of_vehicle)(NumericalID veh_id);


// returns the name of the edge in a char array, given its Numerical ID
EXTERN libLaneFreePlugin_EXPORT char* (* get_edge_name)(NumericalID edge_id);


// returns the length of a given edge id
EXTERN libLaneFreePlugin_EXPORT double (* get_edge_length)(NumericalID edge_id);


// returns the width of a given edge id
EXTERN libLaneFreePlugin_EXPORT double (* get_edge_width)(NumericalID edge_id);


// returns all ids in a given edge, based on the edge's id, and ordered according to their longitudinal position
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_ids_in_edge)(NumericalID edge_id);
// returns the size of all ids in a given edge, based on the edge's id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_all_ids_in_edge_size)(NumericalID edge_id);


// apply longitudinal acceleration accel_x m/s^2 and lateral acceleration accel_y m/s^2 to the vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT void (* apply_acceleration)(NumericalID veh_id, double accel_x, double accel_y);


// returns the longitudinal speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT double (* get_speed_x)(NumericalID veh_id);


// returns the longitudinal speed in m/s of vehicle with id veh_id (positive when moving towards the left boundary)
EXTERN libLaneFreePlugin_EXPORT double (* get_speed_y)(NumericalID veh_id);


// returns the desired speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT double (* get_desired_speed)(NumericalID veh_id);

// sets the desired speed in m/s of vehicle with id veh_id
EXTERN libLaneFreePlugin_EXPORT void (* set_desired_speed)(NumericalID veh_id, double new_des_speed);


// returns the longitudinal position of a vehicle based on the local coordinates (according to the road the vehicle is now) (x position corresponds to the distance of the vehicle's center from the road's origin point)
EXTERN libLaneFreePlugin_EXPORT double (* get_position_x)(NumericalID veh_id);


// returns the lateral position of a vehicle based on the local coordinates (according to the road the vehicle is now) (y position corresponds to the distance of the vehicle's center from the right road boundary)
EXTERN libLaneFreePlugin_EXPORT double (* get_position_y)(NumericalID veh_id);


// returns the relative longitudinal distance of a vehicle with respect to an ego vehicle
EXTERN libLaneFreePlugin_EXPORT double (* get_relative_distance_x)(NumericalID ego_id, NumericalID other_id);


// returns the relative longitudinal position of a vehicle with respect to an ego vehicle
EXTERN libLaneFreePlugin_EXPORT double (* get_relative_position_x)(NumericalID ego_id, NumericalID other_id);


// returns the relative lateral distance of a vehicle with respect to an ego vehicle
EXTERN libLaneFreePlugin_EXPORT double (* get_relative_distance_y)(NumericalID ego_id, NumericalID other_id);


// determines if a given vehicle will be transported at the beginning of the road edge it currently is into, based on the value of circular (true:enables the behavior, false:disables the behavior)
EXTERN libLaneFreePlugin_EXPORT void (* set_circular_movement)(NumericalID veh_id, bool circular);


// returns the type of a given vehicle id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_veh_type_id)(NumericalID veh_id);


// returns the name of a given type id
EXTERN libLaneFreePlugin_EXPORT char* (* get_veh_type_name)(NumericalID veh_id);


// returns the length of a given vehicle id
EXTERN libLaneFreePlugin_EXPORT double (* get_veh_length)(NumericalID veh_id);

// returns the width of a given vehicle id
EXTERN libLaneFreePlugin_EXPORT double (* get_veh_width)(NumericalID veh_id);


// returns 1 if the vehicle is currently on an acceleration lane (should be specified accordingly in the network file) and needs to merge
EXTERN libLaneFreePlugin_EXPORT int (* am_i_on_acceleration_lane)(NumericalID veh_id);


// returns the ids of front vehicles that may be located beyond the veh_id's road edge, according to the front distance provided, and based on its routing. 
// you need "cross_edge=1" to look for vehicles beyond the current edge, and pass the address of a variable to "neighbors_size" in order to acquire the size of the resulting array.
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_neighbor_ids_front)(NumericalID veh_id, double front_distance, int cross_edge, size_t* neighbors_size);


// returns the ids of vehicles one the back that may be located before the veh_id's road edge, according to the back distance provided, and based on its routing. 
// you need "cross_edge=1" to look for vehicles beyond the current edge, and pass the address of a variable to "neighbors_size" in order to acquire the size of the resulting array.
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_all_neighbor_ids_back)(NumericalID veh_id, double back_distance, int cross_edge, size_t* neighbors_size);


// returns the time-step length
EXTERN libLaneFreePlugin_EXPORT double (* get_time_step_length)();


//returns the current time-step
EXTERN libLaneFreePlugin_EXPORT int (* get_current_time_step)();


// returns the seed of SUMO (is defined in the sumocfg file)
EXTERN libLaneFreePlugin_EXPORT int (* get_seed)();


// insert a new vehicle (route_id and type_id need to be defined in the scenario tested), use_global_coordinates is only relevant to the bicycle model, and will be disregarded otherwise
EXTERN libLaneFreePlugin_EXPORT NumericalID(* insert_new_vehicle)(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y, double theta, int use_global_coordinates);


// returns the ids of the detectors
EXTERN libLaneFreePlugin_EXPORT NumericalID* (* get_detectors_ids)();
// returns the size of the ids of the detectors
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_detectors_size)();


// returns the detector's name, based on the detector id
EXTERN libLaneFreePlugin_EXPORT char* (* get_detector_name)(NumericalID detector_id);


// returns the values of all detectors (number of vehicles for each detector) (NOTE: every time we obtain a measurement from detectors, they are reseted)
EXTERN libLaneFreePlugin_EXPORT int* (* get_detectors_values)();


// returns the value of a single detector, based on the detector's id (NOTE: every time we obtain a measurement from detectors, they are reseted)
EXTERN libLaneFreePlugin_EXPORT int (* get_detector_value)(NumericalID detector_id);


// returns the value that corresponds to the specified type of a single detector, based on the detector's id (NOTE: every time we obtain a measurement from detectors, they are reseted)
EXTERN libLaneFreePlugin_EXPORT int (* get_detector_value_for_type)(NumericalID detector_id, char* veh_type);


// returns the density of vehicles (veh/km) for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT double (* get_density_on_segment_region_on_edge)(NumericalID edge_id, double segment_start, double segment_end);


// returns the number of vehicles # for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT int (*get_number_of_vehicles_on_segment_region_on_edge)(NumericalID edge_id, double segment_start, double segment_end);


// returns the density of vehicles (veh/km) only on the main highway for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT double (*get_density_on_segment_region_on_edge_only_highway)(NumericalID edge_id, double segment_start, double segment_end);


// returns the number of vehicles only on the main highway for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT int (*get_number_of_vehicles_on_segment_region_on_edge_only_highway)(NumericalID edge_id, double segment_start, double segment_end);


// returns the density of vehicles (veh/km) only on the ramp (either on-ramp or off-ramp) for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT double (*get_density_on_segment_region_on_edge_only_ramp)(NumericalID edge_id, double segment_start, double segment_end);


// returns the number of vehicles only on the ramp (either on-ramp or off-ramp) for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT int (*get_number_of_vehicles_on_segment_region_on_edge_only_ramp)(NumericalID edge_id, double segment_start, double segment_end);


// returns the density of vehicles (veh/km) per segment for a given edge id, and a segment length
// E.g., given an edge with length 1000m and segment length of 100m, this will return a double array of 10 elements, associated with each 100m segment
// if (edge_length modulo segment_length) != 0, then the resulting array will contain an additional segment at the end associated with the remaining distance
EXTERN libLaneFreePlugin_EXPORT double* (* get_density_per_segment_per_edge)(NumericalID edge_id, double segment_length);
// returns the size of the array provided above
EXTERN libLaneFreePlugin_EXPORT int (* get_density_per_segment_per_edge_size)(NumericalID edge_id, double segment_length);


// returns the density of vehicles (veh/km) for a given segment region for a given edge id and for a given type of vehicles
EXTERN libLaneFreePlugin_EXPORT double (* get_density_on_segment_region_on_edge_for_type)(NumericalID edge_id, double segment_start, double segment_end, char* veh_type);


// returns the average speed of vehicles for a given segment region for a given edge id
EXTERN libLaneFreePlugin_EXPORT double (* get_average_speed_on_segment_region_on_edge)(NumericalID edge_id, double segment_start, double segment_end);


// returns the average speed of vehicles for a given segment region for a given edge id and for a given type of vehicles
EXTERN libLaneFreePlugin_EXPORT double (* get_average_speed_on_segment_region_on_edge_for_type)(NumericalID edge_id, double segment_start, double segment_end, char* veh_type);


// returns the global x position of the vehicle (w.r.t. its center point)
EXTERN libLaneFreePlugin_EXPORT double (* get_global_position_x)(NumericalID veh_id);


// returns the global y position of the vehicle (w.r.t. its center point)
EXTERN libLaneFreePlugin_EXPORT double (* get_global_position_y)(NumericalID veh_id);


// returns the destination edge of the specific vehicle id
EXTERN libLaneFreePlugin_EXPORT NumericalID (* get_destination_edge_id)(NumericalID veh_id);


// returns the origin edge of the specific vehicle id
EXTERN libLaneFreePlugin_EXPORT NumericalID(* get_origin_edge_id)(NumericalID veh_id);


// returns the subsequent edge id of the vehicle. In case of error (also displays error message), or if the vehicle is already at the destination edge, it returns -1.
EXTERN libLaneFreePlugin_EXPORT NumericalID(* get_next_edge_id)(NumericalID veh_id);


// returns the previous edge id of the vehicle.  In case of error (also displays error message), or if the vehicle is at the origin edge, it returns -1.
EXTERN libLaneFreePlugin_EXPORT NumericalID(* get_previous_edge_id)(NumericalID veh_id);


// returns the current orientation of the vehicle, in radians, with respect to the residing road (or global if controller is on global coordinates)
EXTERN libLaneFreePlugin_EXPORT double (* get_veh_orientation)(NumericalID veh_id);


// apply control for vehicles adhering to the bicycle model by providing the F, and delta values for vehicle with numerical id veh_id
EXTERN libLaneFreePlugin_EXPORT void (* apply_control_bicycle_model)(NumericalID veh_id, double F, double delta);


// return the speed of vehicle with numerical id veh_id which adheres to the bicycle model
EXTERN libLaneFreePlugin_EXPORT double (* get_speed_bicycle_model)(NumericalID veh_id);


// set global control on or off based on the value of use_global_coordinates (1 or 0 respectively) (only for the bicycle model vehicles)
EXTERN libLaneFreePlugin_EXPORT void (* set_global_coordinate_control)(NumericalID veh_id, int use_global_coordinates);


// returns the execution time (in seconds) of the previous call for the simulation_step function
EXTERN libLaneFreePlugin_EXPORT double (* get_last_step_time)();


// returns the execution time (in seconds) of the previous step (disregarding the execution time for the simulation_step function, i.e., execution time for the SUMO application)
EXTERN libLaneFreePlugin_EXPORT double (* get_last_step_app_time)();


// Provide the associated route_name with a char array, and an array of epsilon values to change the left boundary of route_name online (and the size of the double array), according to the epsilon_array values
// epsilon_array should have the same size with the defined leftBoundaryLevelPoints for this route.
// Online adjustment of the left boundary for a specific leftBoundaryLevelPoint, and epsilon_value is as follows:
// leftBoundaryAdjusted = rightBoundaryConstant + epsilon_value * (leftBoundaryLevelPoint - rightBoundaryConstant)
// E.g., for epsilon_value=1, the left boundary will not change.
// All epsilon values should be within the range 0 < epsilon_value < 2
EXTERN libLaneFreePlugin_EXPORT void (* set_epsilon_left_boundary)(char* route_name, double* epsilon_array, size_t epsilon_array_size);


// returns a double array with veh densities (veh/km) associated with the segments formed by the left boundary, providing the associated route_name with a char array
EXTERN libLaneFreePlugin_EXPORT double* (*get_density_left_boundary_segments)(char* route_name, size_t* number_of_segments);


// calculates the lateral distance from left and right road boundaries for veh_id, at longitudinal_distance_x (vehicle can also observe upstream with negative values) and lateral_distance_y
// regarding the boundaries, it calculates the boundaries's distances (with left_boundary_distance, right_boundary_distance variables) and first derivative (with left_boundary_speed, right_boundary_speed variables)
// if speed information is not useful, one can simply place NULL pointers to the respective arguments (left_boundary_speed, right_boundary_speed)
// for speed information the argument veh_longitudinal_speed provides the longitudinal speed of the vehicle for the calculation of left_boundary_speed, right_boundary_speed. 
// if veh_longitudinal_speed is a NULL pointer, then the current speed of the vehicle will be choosed for these calculations
EXTERN libLaneFreePlugin_EXPORT void (* get_distance_to_road_boundaries_at)(NumericalID veh_id, double longitudinal_distance_x, double lateral_distance_y, double* left_boundary_distance, double* right_boundary_distance, double* left_boundary_speed, double* right_boundary_speed, double* veh_longitudinal_speed);


// is called once before the first time-step
EXTERN libLaneFreePlugin_EXPORT void simulation_initialize();

// is called in every time-step
EXTERN libLaneFreePlugin_EXPORT void simulation_step();

// is called when the simulation ends
EXTERN libLaneFreePlugin_EXPORT void simulation_finalize();


// is called when a new vehicle enters
EXTERN libLaneFreePlugin_EXPORT void event_vehicle_enter(NumericalID veh_id);

// is called when a new vehicle exits
// if has_arrived==1, then vehicle is out of the network. Otherwise, vehicle is still in the network when simulation was terminated
EXTERN libLaneFreePlugin_EXPORT void event_vehicle_exit(NumericalID veh_id, int has_arrived);


// is called when two vehicles collide
EXTERN libLaneFreePlugin_EXPORT void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2);


// is called when a vehicle exceeds the road boundaries
EXTERN libLaneFreePlugin_EXPORT void event_vehicle_out_of_bounds(NumericalID veh_id);
