#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include "LaneFree_win.h"
#include "libLaneFreePlugin_Export.h"
#endif

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define MAX_ACCEL 2
#define MAX_DECEL 3
#define MAX_LAT_ACCEL 1.5
#define RSS_MAX_ACCEL 2
#define RSS_MAX_DECEL 2
#define RSS_LAT_ACCEL 1
#define RSS_MU 1
#define RSS_RHO_DELAY 0.5

#define RSS_USE_ANGLE 1
#define RSS_MAX_VEHS_FRONT 5
#define RSS_MAX_VEHS_BACK 5
#define RSS_MAX_VEHS 10
#define RSS_MAX_DIST 50

#define RSS_EPSILON_WEIGHT 0.005
//we could later on define the hash_table_size dynamically, i.e., make it increase if needed
#define HASH_TABLE_SIZE 1500

typedef struct{
	NumericalID veh_id;
	double time_long;
	double time_lat;
}dangerous_veh;

typedef struct {
	NumericalID veh_id;
	dangerous_veh* vehs_in_danger;
}veh_memory;


veh_memory* vehs_danger[HASH_TABLE_SIZE];

size_t vehs_size_all;
size_t* vehs_danger_size;

// Code for hash table adapted from: https://www.tutorialspoint.com/data_structures_algorithms/hash_table_program_in_c.htm
int hashCode(NumericalID veh_id) {
	return veh_id % HASH_TABLE_SIZE;
}

veh_memory* search_hash_table(NumericalID veh_id) {
	//get the hash 
	int hashIndex = hashCode(veh_id);

	//move in array until an empty 
	while (vehs_danger[hashIndex] != NULL) {

		if (vehs_danger[hashIndex]->veh_id == veh_id)
			return vehs_danger[hashIndex];

		//go to next cell
		++hashIndex;

		//wrap around the table
		hashIndex %= HASH_TABLE_SIZE;
	}

	return NULL;
}

void insert_veh_hash_table(NumericalID veh_id) {



	//get the hash 
	int hashIndex = hashCode(veh_id);
	int counter = 0;
	//move in array until an empty or deleted cell
	while (vehs_danger[hashIndex] != NULL && vehs_danger[hashIndex]->veh_id != -1) {
		//go to next cell
		++hashIndex;

		//wrap around the table
		hashIndex %= HASH_TABLE_SIZE;
		counter++;
		if (counter > HASH_TABLE_SIZE) {
			printf("Hash table full!\n");
			return;
		}
	}
	if (vehs_danger[hashIndex] == NULL) {
		printf("Error while inserting vehicle to the hash table!\n");
		return;
	}
	vehs_danger[hashIndex]->veh_id = veh_id;
	dangerous_veh* dvehs = (dangerous_veh*)malloc(sizeof(dangerous_veh) * RSS_MAX_VEHS);
	if (dvehs == NULL) {
		printf("Error while allocating memory!\n");
		return;
	}
	for (int i = 0; i < RSS_MAX_VEHS; i++) {
		dvehs[i].veh_id = -1;
		dvehs[i].time_long = -1;
		dvehs[i].time_lat = -1;		
	}
	vehs_danger[hashIndex]->vehs_in_danger = dvehs;
}

void delete_veh_hash_table(NumericalID veh_id) {
	
	//get the hash 
	int hashIndex = hashCode(veh_id);
	int counter = 0;
	//move in array until an empty
	while (vehs_danger[hashIndex] != NULL && vehs_danger[hashIndex]->veh_id != -1) {

		if (vehs_danger[hashIndex]->veh_id == veh_id) {
			vehs_danger[hashIndex]->veh_id = -1;
			free(vehs_danger[hashIndex]->vehs_in_danger);

			//assign a dummy item at deleted position
			return;
		}

		//go to next cell
		++hashIndex;

		//wrap around the table
		hashIndex %= HASH_TABLE_SIZE;

		counter++;
		if (counter > HASH_TABLE_SIZE) {
			printf("Vehicle to be deleted from hashmap not found!\n");
			return;
		}
	}

	return;
}

void simulation_initialize(){
	vehs_size_all = 0;

	for (int i = 0; i < HASH_TABLE_SIZE; i++) {
		vehs_danger[i] = (veh_memory*)malloc(sizeof(veh_memory));
		if (vehs_danger[i] == NULL) {
			printf("Memory could not be allocated for hash table!\n");
			return;
		}
		vehs_danger[i]->veh_id = -1;
		vehs_danger[i]->vehs_in_danger = NULL;
	}

	//initialize srand with the same seed as sumo
	srand(get_seed());

	//insert 20 vehicles
	int n_init = 40;

	//malloc vehs_danger for n_init vehicles, and also init each slot for a predefined size. Then each size will correspond to RSS_MAX_VEHS for each one, i.e., max dangerous vehicles. init with veh_id = -1
		
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



double longitudinal_distance_min(double v_rear, double v_front, double rho_delay, double a_max_accel, double a_min_brake, double a_max_brake) {
	double v_max_rear = v_rear + rho_delay * a_max_accel;
	double max_dist_rear = v_rear * rho_delay + 0.5 * a_max_accel * (pow(rho_delay, 2))+(pow(v_max_rear,2)/(2*a_min_brake));
	double max_dist_front = pow(v_front, 2) / (2 * a_max_brake);
	return max_dist_rear - max_dist_front;
}

//we consider that positive speed/acceleration is towards the right, and vice versa
double lateral_distance_min(double mu, double v_lat_left, double v_lat_right, double rho_delay, double a_lat_max_accel, double a_lat_min_brake) {
	double v_lat_max_left = v_lat_left + rho_delay * a_lat_max_accel;
	double v_lat_max_right = v_lat_right - rho_delay * a_lat_max_accel;
	double max_dist_left = ((v_lat_left + v_lat_max_left) * rho_delay) / 2 + pow(v_lat_max_left, 2) / (2 * a_lat_min_brake);
	double max_dist_right = ((v_lat_right + v_lat_max_right) * rho_delay) / 2 + pow(v_lat_max_right, 2) / (2 * a_lat_min_brake);
	return mu + max_dist_left-max_dist_right;
}

//returns 3 if this is a dangerous situation both longitudinaly and laterally, 2 if only laterally, 1 if only longitudinally, else 0 (also returns -1 if ego is in front)
int rss_detect_danger(double dx, double dy, double vx_rear, double vy_rear, double vx_front, double vy_front, double len_rear, double width_rear, double len_front, double width_front, double* longitudinal_safe_dist, double* lateral_safe_dist) {

	if (dx < 0) {
		printf("Cars are opposite!\n");
		return -1;
	}

	double dx_act = dx - (len_rear + len_front) / 2;

	

	*longitudinal_safe_dist = longitudinal_distance_min(vx_rear, vx_front, RSS_RHO_DELAY, MAX_ACCEL, RSS_MAX_DECEL, MAX_DECEL);

	
	double dy_act = fabs(dy) - (width_rear + width_front) / 2;

	if (dy_act < 0) {
		printf("Cars are colliding lateraly!\n");
		return -1;
	}
	double vy_left, vy_right;
	if (dy > 0) {
		vy_left = -vy_front;
		vy_right = -vy_rear;
	}
	else {
		vy_left = -vy_rear;
		vy_right = -vy_front;
	}
	*lateral_safe_dist = lateral_distance_min(RSS_MU,vy_left, vy_right, RSS_RHO_DELAY, RSS_LAT_ACCEL, RSS_LAT_ACCEL);
	int long_danger = dx_act < *longitudinal_safe_dist;
	int lat_danger = dy_act < *lateral_safe_dist;
	return long_danger + 2 * lat_danger;
}


void proper_response(NumericalID veh_id_j, NumericalID veh_id_j_neighbor, dangerous_veh* vehs_danger_j, double current_time, double dx, double dy, double vx_j, double vy_j, double vx_j_n, double vy_j_n, double len_j, double width_j, double len_j_n, double width_j_n, double long_safe_dist, double lat_safe_dist) {
	int danger;
	if (dx >= 0) {
		danger = rss_detect_danger(dx, dy, vx_j, vy_j, vx_j_n, vy_j_n, len_j, width_j, len_j_n, width_j_n, &long_safe_dist, &lat_safe_dist);
	}
	else {
		danger = rss_detect_danger(-dx, -dy, vx_j_n, vy_j_n, vx_j, vy_j, len_j_n, width_j_n, len_j, width_j, &long_safe_dist, &lat_safe_dist);
	}
	
	int v_i_j_n = -1;
	int v_i_j_avail = -1;
	for (int v_i_j = 0; v_i_j < RSS_MAX_VEHS; v_i_j++) {
		if (v_i_j_avail == -1 && vehs_danger_j[v_i_j].veh_id == -1) {
			v_i_j_avail = v_i_j;
		}
		if (veh_id_j_neighbor == vehs_danger_j[v_i_j].veh_id) {
			v_i_j_n = v_i_j;
		}
		if (v_i_j_n != -1 && v_i_j_avail != -1) {
			break;
		}
	}
	double dx_act, dy_act, angle;
	if (danger) {
		//check if we already have this vehicle listed
		//if not, assign it
		if (v_i_j_n == -1) {
			if (v_i_j_avail == -1) {
				printf("Error, No available placement in vehicle array!\n");
				return;
			}
			v_i_j_n = v_i_j_avail;
			vehs_danger_j[v_i_j_n].veh_id = veh_id_j_neighbor;

		}
		if (danger == 1) {
			//only longitudinal danger


			//update time long. if it was not in danger before
			if (vehs_danger_j[v_i_j_n].time_long == -1) {
				vehs_danger_j[v_i_j_n].time_long = current_time;
			}

			//update the time_lat (reset if it had a value previously)
			vehs_danger_j[v_i_j_n].time_lat = -1;
		}
		else if (danger == 2) {
			//only lateral danger


			//update time long. if it was not in danger before
			if (vehs_danger_j[v_i_j_n].time_lat == -1) {
				vehs_danger_j[v_i_j_n].time_lat = current_time;
			}

			//update the time long (reset if it had a value previously)
			vehs_danger_j[v_i_j_n].time_long = -1;

		}
		else {
			dx_act = dx - (len_j + len_j_n) / 2;
			worst_front_weight = fmax(worst_front_weight, long_safe_dist / (dx_act + RSS_EPSILON_WEIGHT));
			dy_act = fabs(dy) - (width_j + width_j_n) / 2;
			if (dy > 0) {
				//using fabs on dy_act, the weight will also be increased when the vehicles are already colliding
				worst_left_weight = fmax(worst_left_weight, lat_safe_dist / (fabs(dy_act) + RSS_EPSILON_WEIGHT));
			}
			else {
				worst_right_weight = fmax(worst_right_weight, lat_safe_dist / (fabs(dy_act) + RSS_EPSILON_WEIGHT));
			}

			//this is actually a potentially dangerous situation
			if (vehs_danger_j[v_i_j_n].time_lat == -1) {
				vehs_danger_j[v_i_j_n].time_lat = current_time;
			}
			if (vehs_danger_j[v_i_j_n].time_long == -1) {
				vehs_danger_j[v_i_j_n].time_long = current_time;
			}


			if (!RSS_USE_ANGLE) {
				//we will either react according to the paper, i.e., only longitudinally or laterally, according to the time
				//if equality holds, we react in both dimentions
				if (vehs_danger_j[v_i_j_n].time_long <= vehs_danger_j[v_i_j_n].time_lat) {
					//react laterally
					if (dy > 0) {
						uy_left_bound = -RSS_LAT_ACCEL;
					}
					else {
						uy_right_bound = RSS_LAT_ACCEL;
					}
				}
				if (vehs_danger_j[v_i_j_n].time_long >= vehs_danger_j[v_i_j_n].time_lat) {
					//react longitudinally
					ux_front_bound = -RSS_MAX_DECEL;
				}




			}
			else {
				//or according to our technique, using the angle
				//we find the angle according to dx,dy
				angle = atan2(dy, dx);
				ux_front_bound = fmin(-RSS_MAX_DECEL * cos(angle), ux_front_bound);
				if (dy > 0) {
					uy_left_bound = fmin(-RSS_LAT_ACCEL * sin(angle), uy_left_bound);
				}
				else {
					uy_right_bound = fmax(-RSS_LAT_ACCEL * sin(angle), uy_right_bound);
				}
			}

		}
	}
	else {
		//no danger, if j_neighbor is in the dangerous vehicles array, remove it
		if (v_i_j_n != -1) {
			vehs_danger_j[v_i_j_n].veh_id = -1;
			vehs_danger_j[v_i_j_n].time_long = -1;
			vehs_danger_j[v_i_j_n].time_lat = -1;
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

	//Check the density per road per segment
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();

	
	//For larger networks, you may control vehicles based on the road edge they are in
	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;
	
	double vx, accel;
	int j_neighbor, counter, danger,v_i_j, v_i_j_n, v_i_j_avail;
	double dx, dy, vx_j, vy_j, vx_j_n, vy_j_n, len_j, len_j_n, width_j, width_j_n, long_safe_dist, lat_safe_dist;
	double dx_act, dy_act;
	double ux_front_bound, ux_back_bound, uy_left_bound, uy_right_bound, angle;
	double worst_front_weight, worst_back_weight, worst_left_weight, worst_right_weight;
	veh_memory* veh_m_j;
	dangerous_veh* vehs_danger_j;
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
				//vname = get_vehicle_name(ids_in_edge[j]);
				vx_j = get_speed_x(ids_in_edge[j]);
				vy_j = get_speed_y(ids_in_edge[j]);
				len_j = get_veh_length(ids_in_edge[j]);
				width_j = get_veh_width(ids_in_edge[j]);
				veh_m_j = search_hash_table(ids_in_edge[j]);
				vehs_danger_j = veh_m_j->vehs_in_danger;
				counter = 0;
				ux_front_bound = MAX_ACCEL;
				ux_back_bound = -MAX_ACCEL;
				uy_left_bound = MAX_LAT_ACCEL;
				uy_right_bound = -MAX_LAT_ACCEL;
				worst_front_weight = -1;
				worst_left_weight = -1;
				worst_back_weight = -1;
				worst_right_weight = -1;
				//neighbors downstream
				for (j_neighbor = j+ 1;; j_neighbor++) {
					j_neighbor = j_neighbor % n_edge_ids;
					if (j_neighbor == j || counter==RSS_MAX_VEHS_FRONT) {
						break;
					}
					dx = get_relative_distance_x(ids_in_edge[j],ids_in_edge[j_neighbor]);
					if (dx > RSS_MAX_DIST || dx < 0) {
						break;
					}
					dy = get_relative_distance_y(ids_in_edge[j], ids_in_edge[j_neighbor]);
					vx_j_n = get_speed_x(ids_in_edge[j_neighbor]);
					vy_j_n = get_speed_y(ids_in_edge[j_neighbor]);
					len_j_n = get_veh_length(ids_in_edge[j_neighbor]);
					width_j_n = get_veh_width(ids_in_edge[j_neighbor]);
					
					danger = rss_detect_danger(dx, dy, vx_j, vy_j, vx_j_n, vy_j_n, len_j, width_j, len_j_n, width_j_n,&long_safe_dist,&lat_safe_dist);
					v_i_j_n = -1;
					v_i_j_avail = -1;
					for (v_i_j = 0; v_i_j < RSS_MAX_VEHS; v_i_j++) {
						if (v_i_j_avail == -1 && vehs_danger_j[v_i_j].veh_id == -1) {
							v_i_j_avail = v_i_j;
						}
						if (ids_in_edge[j_neighbor] == vehs_danger_j[v_i_j].veh_id) {
							v_i_j_n = v_i_j;
						}
						if (v_i_j_n != -1 && v_i_j_avail != -1) {
							break;
						}
					}

					if (danger) {
						//check if we already have this vehicle listed
						//if not, assign it
						if (v_i_j_n == -1) {
							if (v_i_j_avail == -1) {
								printf("Error, No available placement in vehicle array!\n");
								return;
							}
							v_i_j_n = v_i_j_avail;
							vehs_danger_j[v_i_j_n].veh_id = ids_in_edge[j_neighbor];

						}
						if (danger == 1) {
							//only longitudinal danger
							

							//update time long. if it was not in danger before
							if (vehs_danger_j[v_i_j_n].time_long == -1) {
								vehs_danger_j[v_i_j_n].time_long = t * TS;
							}

							//update the time_lat (reset if it had a value previously)
							vehs_danger_j[v_i_j_n].time_lat = -1;
						}
						else if (danger == 2) {
							//only lateral danger


							//update time long. if it was not in danger before
							if (vehs_danger_j[v_i_j_n].time_lat == -1) {
								vehs_danger_j[v_i_j_n].time_lat = t * TS;
							}

							//update the time long (reset if it had a value previously)
							vehs_danger_j[v_i_j_n].time_long = -1;

						}
						else {
							dx_act = dx - (len_j + len_j_n) / 2;
							worst_front_weight = fmax(worst_front_weight, long_safe_dist / (dx_act + RSS_EPSILON_WEIGHT));
							dy_act = fabs(dy) - (width_j + width_j_n) / 2;
							if (dy > 0) {
								//using fabs on dy_act, the weight will also be increased when the vehicles are already colliding
								worst_left_weight = fmax(worst_left_weight, lat_safe_dist / (fabs(dy_act) + RSS_EPSILON_WEIGHT));
							}
							else {
								worst_right_weight = fmax(worst_right_weight, lat_safe_dist / (fabs(dy_act) + RSS_EPSILON_WEIGHT));
							}

							//this is actually a potentially dangerous situation
							if (vehs_danger_j[v_i_j_n].time_lat == -1) {
								vehs_danger_j[v_i_j_n].time_lat = t * TS;
							}
							if (vehs_danger_j[v_i_j_n].time_long == -1) {
								vehs_danger_j[v_i_j_n].time_long = t * TS;
							}


							if (!RSS_USE_ANGLE) {
								//we will either react according to the paper, i.e., only longitudinally or laterally, according to the time
								//if equality holds, we react in both dimentions
								if (vehs_danger_j[v_i_j_n].time_long <= vehs_danger_j[v_i_j_n].time_lat) {
									//react laterally
									if (dy > 0) {
										uy_left_bound = -RSS_LAT_ACCEL;
									}
									else {
										uy_right_bound = RSS_LAT_ACCEL;
									}
								}
								if (vehs_danger_j[v_i_j_n].time_long >= vehs_danger_j[v_i_j_n].time_lat) {
									//react longitudinally
									ux_front_bound = -RSS_MAX_DECEL;
								}
								
								

								
							}
							else {
								//or according to our technique, using the angle
								//we find the angle according to dx,dy
								angle = atan2(dy, dx);
								ux_front_bound = fmin(-RSS_MAX_DECEL * cos(angle),ux_front_bound);
								if (dy > 0) {
									uy_left_bound = fmin(-RSS_LAT_ACCEL * sin(angle),uy_left_bound);
								}
								else {
									uy_right_bound = fmax(-RSS_LAT_ACCEL * sin(angle), uy_right_bound);
								}
							}
							
						}
					}
					else {
						//no danger, if j_neighbor is in the dangerous vehicles array, remove it
						if (v_i_j_n != -1) {
							vehs_danger_j[v_i_j_n].veh_id = -1;
							vehs_danger_j[v_i_j_n].time_long = -1;
							vehs_danger_j[v_i_j_n].time_lat = -1;
						}
					}
				}
				counter = 0;
				//neighbors downstream
				for (j_neighbor = j_neighbor - 1;; j_neighbor--) {
					j_neighbor = j_neighbor % n_edge_ids;
					if (j_neighbor == j) {
						break;
					}
					dx = get_relative_distance_x(ids_in_edge[j], ids_in_edge[j_neighbor]);
					if (dx < -RSS_MAX_DIST || dx > 0) {
						break;
					}



					if (!RSS_USE_ANGLE) {

					}
				}
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

	//insert to the hash table
	insert_veh_hash_table(veh_id);
}

void event_vehicle_exit(NumericalID veh_id){
	char* vname1 = get_vehicle_name(veh_id);
	printf("Vehicle %s exited at time %.2f, at pos:%f.\n",vname1, get_current_time_step()*get_time_step_length(), get_position_x(veh_id));
	
	//delete from hash table
	delete_veh_hash_table(veh_id);
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
