
#include <stdlib.h>
#include "math.h"


#include "LaneFree.h"


#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35

#include <stdio.h>

void simulation_initialize(){

	//initialize srand with the same seed as sumo
	srand(get_seed());

	//insert 20 vehicles
	int n_init = 0;
		
	double x_incr=25, y_incr=2.5, vx_incr=5;
	double x_val=x_incr, y_val=y_incr, vx_val=vx_incr;
	int virtual_lanes = 3;
	double width=10;


	char veh_name[40];
	//route_id and type_id should be defined in the scenario we are running
	char route_id[20]="route0";
	char type_id[20]="1";
	NumericalID v_id;
	for(int i=0;i<n_init;i++){
		
		sprintf(type_id, "%d", i%8+1);
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
	NumericalID* myids = get_lane_free_ids();
	NumericalID n_myids = get_lane_free_ids_size();
	double pos_x, pos_y, speed, speed_y, des_speed, ux, uy, length, width, TS = get_time_step_length();
	int t = get_current_time_step();
	int i, j;
	char* vname;
	//printf("timestep:%d\n", get_current_time_step());
	
	for (i = 0; i < n_myids; i++) {
		

		pos_x = get_position_x(myids[i]);
		pos_y = get_position_y(myids[i]);
		speed = get_speed_x(myids[i]);
		length = get_veh_length(myids[i]);
		width = get_veh_width(myids[i]);
		des_speed = get_desired_speed(myids[i]);
		
		speed_y = get_speed_y(myids[i]);
		uy = erfc(speed_y)-1;
		ux = erfc(speed-des_speed)-1;
		apply_acceleration(myids[i], ux, uy);
		vname = get_vehicle_name(myids[i]);
		//printf("%s\n",vname);

		//Check if the positions follow the equations of motion
		// printf("vehid: %s in pos: %f, %f \n",vname, pos_x, pos_y );
		// pos_x = pos_x + speed*TS + 0.5*ux*TS*TS;
		// pos_y = pos_y + speed_y*TS + 0.5*uy*TS*TS;
		// printf("next pos: %f, %f \n", pos_x, pos_y );

	}



	NumericalID* detector_ids = get_detectors_ids();
	int* detector_values = get_detectors_values();

	NumericalID detectors_size = get_detectors_size();

	char* detector_name;
	for (j = 0; j < detectors_size; j++) {
		detector_name = get_detector_name(detector_ids[j]);
		printf("detector:%s count:%d\n", detector_name, detector_values[j]);
	}

	//Check the density per road per segment
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	double* density_per_edge;
	int size;
	double segment_length = 100; //in meters
	for (i = 0; i < n_myedges; i++) {
		density_per_edge = get_density_per_segment_per_edge(myedges[i], segment_length);
		if (density_per_edge != NULL) {
			size = get_density_per_segment_per_edge_size(myedges[i], segment_length);
			//printf("Edge id %lld\nDensity per segment:", myedges[i]);
			//for (j = 0; j < size; j++) {
			//	printf("%d,", density_per_edge[j]);
			//}
			//printf("\n");

			
		}

	}
	
	//For larger networks, you may control vehicles based on the road edge they are in
	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;
	
	double vx, accel;
	
	for(i=0;i<n_myedges;i++){
	 	//printf("edge id: %lld\n", myedges[i]);
	 	n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		length = get_edge_length(myedges[i]);
		width = get_edge_width(myedges[i]);
	 	if(n_edge_ids>0){
			//vehicles are ordered
	 		ids_in_edge = get_all_ids_in_edge(myedges[i]);
			//printf("Vehicles in edge with id %lld:", myedges[i]);
			for (j = 0; j < n_edge_ids; j++) {
				vname = get_vehicle_name(ids_in_edge[j]);
				//printf("%s\t",vname);
			}
			//printf("\n\n");
	 	}
	 }
	
	
}

void simulation_finalize(){
	
}


void event_vehicle_enter(NumericalID veh_id){
	
	//set_desired_speed(veh_id, rand()%(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED);
	set_desired_speed(veh_id, (double)(rand() % (int)(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED));
	//char* vname1 = get_vehicle_name(veh_id);
	// printf("Vehicle %s entered with speed %f.\n",vname1,get_speed_x(veh_id));

	//make the vehicles emulate a ring road scenario
	set_circular_movement(veh_id, true);
	
}

// If has_arrived==1, then vehicle is out of the network. Otherwise, vehicle is still in the network when simulation was terminated
void event_vehicle_exit(NumericalID veh_id, int has_arrived){
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

void event_vehicle_out_of_bounds(NumericalID veh_id) {
	char* vname1 = get_vehicle_name(veh_id);	
	printf("Vehicle %s is out of bounds at time %.2f, at pos:%f,%f.\n", vname1, get_current_time_step() * get_time_step_length(), get_position_x(veh_id),get_position_y(veh_id));
}
