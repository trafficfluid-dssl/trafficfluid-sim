#include <stdio.h>
#include "math.h"

#if defined(WIN32)
#include <stdlib.h>
#include "LaneFree_win.h"
#include "libLaneFreePlugin_Export.h"
#elif defined(UNIX)
#include "LaneFree_linux.h"
#endif

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35

void simulation_initialize() {

}
void simulation_initialize2(){
	// printf("hey\n");
	srand(get_seed());

	int n_init = 20;
		
	double x_incr=5, y_incr=2.5, vx_incr=5;
	double x_val=x_incr, y_val=y_incr, vx_val=vx_incr;
	int virtual_lanes = 3;
	double width=10;

	char veh_name[40];
	char route_id[20]="route0";
	char type_id[20]="normal_car_35";
	NumericalID v_id;
	for(int i=0;i<n_init;i++){

		
		sprintf(veh_name, "%s_plugin_%d", type_id,(i+1));
		//printf("%s\n",veh_name);
		v_id = insert_new_vehicle(veh_name, route_id, type_id, x_val, y_val, vx_val,0);
		//set_desired_speed(v_id,35);
		y_val = y_val + y_incr;
		if(i%virtual_lanes==(virtual_lanes-1)){
			x_val += x_incr;
			vx_val += vx_incr;
			y_val = y_incr;
		}

	}
	

	
}

void simulation_step() {
	NumericalID* myids = get_lane_free_ids();
	NumericalID n_myids = get_lane_free_ids_size();
	double pos_x, pos_y, speed, speed_y, des_speed, ux, uy, TS = get_time_step_length();
	int t = get_current_time_step();
	
	char* vname;
	
	print_message("timestep:%d\n", get_current_time_step());
	
	for (int i = 0; i < n_myids; i++) {
		

		pos_x = get_position_x(myids[i]);
		pos_y = get_position_y(myids[i]);
		speed = get_speed_x(myids[i]);
		
		des_speed = get_desired_speed(myids[i]);
		
		speed_y = get_speed_y(myids[i]);
		uy = erfc(speed_y)-1;
		ux = erfc(speed-des_speed)-1;
		apply_acceleration(myids[i], ux, uy);
		vname = get_vehicle_name(myids[i]);
		print_message("%s\n",vname);

		//Check if the positions follow the equations of motion
		// print_message("vehid: %s in pos: %f, %f \n",vname, pos_x, pos_y );
		// pos_x = pos_x + speed*TS + 0.5*ux*TS*TS;
		// pos_y = pos_y + speed_y*TS + 0.5*uy*TS*TS;
		// print_message("next pos: %f, %f \n", pos_x, pos_y );

	}



	NumericalID* detector_ids = get_detectors_ids();
	int* detector_values = get_detectors_values();

	NumericalID detectors_size = get_detectors_size();

	char* detector_name;
	for (int j = 0; j < detectors_size; j++) {
		detector_name = get_detector_name(detector_ids[j]);
		print_message("detector:%lld count:%d\n", detector_ids[j], detector_values[j]);
	}

	//Check the density per road per segment
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	int* density_per_edge;
	int size;
	double segment_length = 100; //in meters
	for (int i = 0; i < n_myedges; i++) {
		density_per_edge = get_density_per_segment_per_edge(myedges[i], segment_length);
		if (density_per_edge != NULL) {
			size = get_density_per_segment_per_edge_size(myedges[i], segment_length);
			print_message("Edge id %lld\nDensity per segment:", myedges[i]);
			for (int j = 0; j < size; j++) {
				print_message("\t%d", density_per_edge[j]);
			}
			print_message("\n");

			
		}

	}
	
	//For larger networks, you may control vehicles based on the road edge they are in
	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;
	int i,j;
	double vx, accel;
	
	for(i=0;i<n_myedges;i++){
	 	print_message("edge id: %lld\n", myedges[i]);
	 	n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
	 	if(n_edge_ids>0){
	 		ids_in_edge = get_all_ids_in_edge(myedges[i]);
			for (j = 0; j < n_edge_ids; j++) {
				vname = get_vehicle_name(ids_in_edge[j]);
				print_message("%s in edge\n",vname);
			}
	 	}
	 }
	
	
}

void simulation_finalize(){
	
}


void event_vehicle_enter(NumericalID veh_id){
	int min_speed = 25, max_speed = 35;
	//set_desired_speed(veh_id, rand()%(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED);
	set_desired_speed(veh_id, (double)(rand() % (int)(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED));
	//char* vname1 = get_vehicle_name(veh_id);
	// printf("Vehicle %s entered with speed %f.\n",vname1,get_speed_x(veh_id));
	// if(count<n_init){
	// 	*init_pos_x = x_init[count];
	// 	*init_pos_y = y_init[count];
	// 	*init_speed_x = vx_init[count];
	// 	count++;
	// }
	//free(vname1);
}

void event_vehicle_exit(NumericalID veh_id){
	//char* vname1 = get_vehicle_name(veh_id);
	// printf("Vehicle %s exited.\n",vname1);
	//free(vname1);
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2){
	//char* vname1 = get_vehicle_name(veh_id1);
	//char* vname2 = get_vehicle_name(veh_id2);
	// printf("Collision between %s and %s at timestep: %d, and time: %.1f.\n",vname1, vname2, get_current_time_step(), get_current_time_step()*get_time_step_length());
	//free(vname1);
	//free(vname2);
}
