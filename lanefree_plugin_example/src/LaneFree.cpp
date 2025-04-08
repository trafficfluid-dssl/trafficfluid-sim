#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define DEFINE_VARIABLES

#include "LaneFree.h"



#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35


void simulation_initialize(){

	//initialize srand with the same seed as sumo
	srand(get_seed());
	
	//insert n_init vehicles
	int n_init = 0;

	// define the spacing in the x axis through x_incr, and y axis through y_incr
	// and whether the initial speed vx will be different as we insert more rows through vx_incr 
	double x_incr=25, y_incr=2.5, vx_incr=5;

	// initialize the x,y, vx values for the first vehicle to be inserted
	double x_val=x_incr, y_val=y_incr, vx_val=vx_incr;
	// number of virtual lanes 
	int virtual_lanes = 3;
	// considered road width
	double width=10;

	char veh_name[40];
	//route_id and type_id should be defined in the scenario we are running
	char route_id[20]="route0";
	char type_id[20]="typedist1";
	NumericalID v_id;
	for(int i=0;i<n_init;i++){
		sprintf(veh_name, "%s_plugin_%d", type_id,(i+1));		
		v_id = insert_new_vehicle(veh_name, route_id, type_id, x_val, y_val, vx_val,0,0,0);
		printf("%s inserted\n", veh_name);
		y_val = y_val + y_incr;
		if(i%virtual_lanes==(virtual_lanes-1)){
			x_val += x_incr;
			if (vx_val < 35) {
				vx_val += vx_incr;
			}			
			y_val = y_incr;
		}
	}	
}

void simulation_step() {

	double pos_x, pos_y, speed, speed_y, des_speed, ux, uy, length, width, TS = get_time_step_length();
	int t = get_current_time_step();
	int i, j;
	char* vname;
	//printf("Print Check!\n");
	printf("timestep:%d\n", get_current_time_step());
	double left_boundary_dist, right_boundary_dist;
	
	
	//one coould grab all vehicles' information in the road, with the following two functions
	//NumericalID* myids = get_lane_free_ids();
	//NumericalID n_myids = get_lane_free_ids_size();
	// and iterate over each vehicle below.
	// However, 
	//for (i = 0; i < n_myids; i++) {	
		// ...		
	//}

	
	int detector_frequency_measurements = 1000; //in time-steps

	if (get_current_time_step() % detector_frequency_measurements == 0) {
		NumericalID* detector_ids = get_detectors_ids();
		int* detector_values = get_detectors_values();

		NumericalID detectors_size = get_detectors_size();

		char* detector_name;
		for (j = 0; j < detectors_size; j++) {
			detector_name = get_detector_name(detector_ids[j]);
			printf("detector:%s count:%d\n", detector_name, detector_values[j]);
		}
	}
	
	
	
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	
	
	//Check the density per road per segment
	//double* density_per_edge;
	//int size;
	//double segment_length = 100; //in meters
	//for (i = 0; i < n_myedges; i++) {
	//	density_per_edge = get_density_per_segment_per_edge(myedges[i], segment_length);
	//	if (density_per_edge != NULL) {
	//		size = get_density_per_segment_per_edge_size(myedges[i], segment_length);
	//		//printf("Edge id %lld\nDensity per segment:", myedges[i]);
	//		//for (j = 0; j < size; j++) {
	//		//	printf("%d,", density_per_edge[j]);
	//		//}
	//		//printf("\n");

	//		
	//	}

	//}
	
	//For larger networks, you may control vehicles based on the road edge they are in
	NumericalID *ids_in_edge;
	NumericalID n_edge_ids;
	
	double vx, accel;

	NumericalID *front_neighbors, *back_neighbors;
	size_t front_neighbors_size, back_neighbors_size;

	double obs_distance = 20;
	for(i=0;i<n_myedges;i++){
	 	//printf("edge id: %lld\n", myedges[i]);
	 	n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		length = get_edge_length(myedges[i]);
		width = get_edge_width(myedges[i]);
	 	if(n_edge_ids>0){
			//vehicles are ordered in this case based on their longitudinal position on the road
	 		ids_in_edge = get_all_ids_in_edge(myedges[i]);
			//printf("Vehicles in edge with id %lld:", myedges[i]);
			for (j = 0; j < n_edge_ids; j++) {
				NumericalID myID = ids_in_edge[j];
				front_neighbors = get_all_neighbor_ids_front(myID, obs_distance, 1, &front_neighbors_size);
				back_neighbors = get_all_neighbor_ids_back(myID, obs_distance, 1, &back_neighbors_size);

				// get our current state
				pos_x = get_position_x(myID);
				pos_y = get_position_y(myID);
				speed = get_speed_x(myID);
				length = get_veh_length(myID);
				width = get_veh_width(myID);
				des_speed = get_desired_speed(myID);

				speed_y = get_speed_y(myID);

				//parse neighbors in front
				for (int k_f = 0; k_f < front_neighbors_size; k_f++) {
					NumericalID neigh_id_front = front_neighbors[k_f];
					// request information regarding this neighbor
					double dx = get_relative_distance_x(myID, neigh_id_front);
					double dy = get_relative_distance_y(myID, neigh_id_front);
					double speed_neigh = get_speed_x(neigh_id_front);
					// etc...
					// depending on the movement strategy design, 
					// users need to develop a proper response to surrounding traffic
				}
	
				//parse neighbors in back
				for (int k_f = 0; k_f < back_neighbors_size; k_f++) {
					NumericalID neigh_id_back = back_neighbors[k_f];
					// request information regarding this neighbor
					double dx = get_relative_distance_x(myID, neigh_id_back);
					double dy = get_relative_distance_y(myID, neigh_id_back);
					double speed_neigh = get_speed_x(neigh_id_back);
					// etc...
					// depending on the movement strategy design, 
					// users need to develop a proper response to surrounding traffic
				}
				
				

				// Below we implement a very simplistic lateral boundary controller that maintains the vehicle within the road boundaries. 
				// This serves as an example to showcase the usage alongside the get_distance_to_road_boundaries_at function. Users can utilize this directly, extend it, or develop something new for their own controller.

				// As a simple measure, we check the distance to the boundary at 50 meters ahead. This could be more elaborate, e.g., based on time headway.
				double longitudinal_distance_to_look = 50;
				get_distance_to_road_boundaries_at(myID, longitudinal_distance_to_look, 0, &left_boundary_dist, &right_boundary_dist, NULL, NULL, NULL);
				double target_y = pos_y;
				double y_const = 0.2;


				// We assume that the lateral space is always large enough, i.e., the vehicle cannot overlap both boundaries at the same time.
				if (left_boundary_dist < width / 2.) {
					// we target the lateral position that will make this condition false, and thus make the vehicle follow the boundary. A small constant term is appended so that the vehicle is not located at the edge of the boundary
					target_y = pos_y - ((width / 2.) - left_boundary_dist) - y_const;
				}
				else if (right_boundary_dist < width / 2.) {
					target_y = pos_y + ((width / 2.) - right_boundary_dist) + y_const;
				}
				double K_p = 1, K_d = 2;

				// simple PD boundary controller
				uy = K_d * (-speed_y) + K_p * (target_y - pos_y);

				// simple controller where longitudinal acceleration targets the desired speed objective
				ux = erfc(speed - des_speed) - 1;

				//apply_acceleration(myID, 0, 0); // default: zero acceleration on both axes
				apply_acceleration(myID, 0, uy); // in the context of our example, we only want to maintain the vehicle within the road boundaries, and not introduce 
				//apply_acceleration(myID, ux, uy); // typical use-case

				vname = get_vehicle_name(myID);
				//printf("%s on the road\n",vname);

				// we can check if the positions follow the equations of motion for the double integrator
				// likewise, one can check for the bicycle model as well
				// printf("vehid: %s in pos: %f, %f \n",vname, pos_x, pos_y );
				// pos_x = pos_x + speed*TS + 0.5*ux*TS*TS;
				// pos_y = pos_y + speed_y*TS + 0.5*uy*TS*TS;
				// printf("next pos: %f, %f \n", pos_x, pos_y );
			}			
	 	}
	}
	


	// For IBC Application
	/*
	int ibc_frequency_update = 100; // in time-steps

	if (get_current_time_step()>0 && (get_current_time_step() % ibc_frequency_update) == 0) {
		
		char route_name [30] = "main_highway"; //need the string information of the route to update the epsilons
		int epsilon_size = 7; // number of leftBoundaryLevelPoints in the .rou.xml file
		
		//array of epsilons (1 is the default value, exactly at the point of road's width)
		double epsilon_array[7] = { 0.95, 1.1, 1.2, 1.3, 1.2, 1.1, 0.95}; 

		
		set_epsilon_left_boundary(route_name, epsilon_array, epsilon_size);

		char route_name_opp[30] = "main_highway_opp";
		double epsilon_array_opp[7];
		double epsilon_threshold = 0.05;
		// simply assign the complementary lateral availability of the road, 
		// accounting for an additional 5% gap between the two opposing boundaries
		for (int e_i = 0; e_i < epsilon_size; e_i++) { 
			epsilon_array_opp[e_i] = 1 + (1 - epsilon_array[e_i]) - epsilon_threshold;
		}

		set_epsilon_left_boundary(route_name_opp, epsilon_array_opp, epsilon_size);

	}
	*/
	
	
}

void simulation_finalize(){
	
}


void event_vehicle_enter(NumericalID veh_id){
		
	set_desired_speed(veh_id, MIN_DESIRED_SPEED + (((double)rand()/RAND_MAX) * (MAX_DESIRED_SPEED - MIN_DESIRED_SPEED)));
	//char* vname1 = get_vehicle_name(veh_id);
	// printf("Vehicle %s entered with speed %f.\n",vname1,get_speed_x(veh_id));

	//make the vehicles emulate a ring road scenario
	set_circular_movement(veh_id, false);
	
}

void event_vehicle_exit(NumericalID veh_id, int has_arrived){
	char* vname1 = get_vehicle_name(veh_id);
	printf("Vehicle %s exited at time %.2f, at pos:%f.\n",vname1, get_current_time_step()*get_time_step_length(), get_position_x(veh_id));
	
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2){
	char vname1[40];
	sprintf(vname1,"%s",get_vehicle_name(veh_id1));
	
	char* vname2 = get_vehicle_name(veh_id2);
	printf("Collision between %s and %s at timestep: %d, and time: %.1f.\n",vname1, vname2, get_current_time_step(), get_current_time_step()*get_time_step_length());
	
}

void event_vehicle_out_of_bounds(NumericalID veh_id) {
	char* vname1 = get_vehicle_name(veh_id);	
	printf("Vehicle %s is out of bounds at time %.2f, at pos:%f,%f.\n", vname1, get_current_time_step() * get_time_step_length(), get_position_x(veh_id),get_position_y(veh_id));
}
