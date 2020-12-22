#include <stdio.h>
#include <stdlib.h>
#define DEFINE_VARIABLES
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree_win.h>

#endif

#include "Controller.h"

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35

//define global simulation variable
sim_t sim_params;

void simulation_initialize(){
	
	

	//initialize srand with the same seed as sumo
	srand(get_seed());

	sim_configure(&sim_params);

	//insert 20 vehicles
	int n_init = 425;
		
	double x_incr=5, y_incr=2.5, vx_incr=5;
	double x_val=x_incr, y_val=y_incr, vx_val=vx_incr;
	int virtual_lanes = 3;
	double width=10;


	char veh_name[40];
	//route_id and type_id should be defined in the scenario we are running
	char route_id[20]="route0";
	char type_id[20]="lane_free_car";
	NumericalID v_id;
	for(int i=0;i<n_init;i++){
				
		sprintf(veh_name, "%s_plugin_%d", type_id,(i+1));
		
		v_id = insert_new_vehicle(veh_name, route_id, type_id, x_val, y_val, vx_val,0);
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
	
	printf("timestep:%d\n", get_current_time_step());
	
	

	//Check the density per road per segment
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	int* density_per_edge;
	int size;
	double segment_length = 100; //in meters
	
	//For larger networks, you may control vehicles based on the road edge they are in
	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;
	char* veh_name;
	double vx, accel;
	
	for(i=0;i<n_myedges;i++){
	 	//printf("edge id: %lld\n", myedges[i]);
	 	n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		length = get_edge_length(myedges[i]);
		width = get_edge_width(myedges[i]);
	 	if(n_edge_ids>0){
			//vehicles are ordered
	 		ids_in_edge = get_all_ids_in_edge(myedges[i]);
			
			for (j = 0; j < n_edge_ids; j++) {
				double fx, fy;
				//veh_name = get_vehicle_name(ids_in_edge[j]);
				//printf("Calculate for vehicle %s.\n", veh_name);
				determine_forces(&sim_params, myedges[i], j, ids_in_edge, n_edge_ids, &fx, &fy);
				//printf("Forces determined.");
				regulate_forces(&sim_params, myedges[i], ids_in_edge[j], &fx, &fy);
				determine_controls(&sim_params, &fx, &fy);
				apply_acceleration(ids_in_edge[j], fx, fy);
			}
			
	 	}
	 }
	
	
}

void simulation_finalize(){
	
}

double drand(double min, double max) {
	double f = (double)rand() / RAND_MAX;
	return min + f * (max - min);
}


void event_vehicle_enter(NumericalID veh_id){
	int min_speed = 25, max_speed = 35;
	
	set_desired_speed(veh_id, drand(MIN_DESIRED_SPEED,MAX_DESIRED_SPEED));
	//char* vname1 = get_vehicle_name(veh_id);
	// printf("Vehicle %s entered with speed %f.\n",vname1,get_speed_x(veh_id));

	//make the vehicles emulate a ring road scenario
	set_circular_movement(veh_id, true);
	
}

void event_vehicle_exit(NumericalID veh_id){
	char* vname1 = get_vehicle_name(veh_id);
	printf("Vehicle %s exited at time %.2f, at pos:%f.\n",vname1, get_current_time_step()*get_time_step_length(), get_position_x(veh_id));
	
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2){
	//char vname1[40];
	//sprintf(vname1,"%s",get_vehicle_name(veh_id1));
	//
	//char* vname2 = get_vehicle_name(veh_id2);
	// printf("Collision between %s and %s at timestep: %d, and time: %.1f.\n",vname1, vname2, get_current_time_step(), get_current_time_step()*get_time_step_length());
	
}


//is called when a vehicle exceeds the road boundaries
void event_vehicle_out_of_bounds(NumericalID veh_id) {

}