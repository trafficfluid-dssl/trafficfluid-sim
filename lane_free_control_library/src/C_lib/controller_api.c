#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>



#include "controller_api.h"

int get_index(size_t id);

//global variable sim_global contains the necessary environment information
sim_env *sim_env_global=NULL;

//returns the x,y longitudinal and lateral position of the corresponding id to the assigned pointers 
void get_position(size_t id, double *x, double *y){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	*x = sim_env_global->x[id_index];
	*y = sim_env_global->y[id_index];	
}

double circular_dx(double dx) {
	if (!sim_env_global->circular){		
		return dx;
	}
    double len = get_road_length();
    if (fabs(dx) <= 0.5*len)
        return dx;
    else
        return (dx >= 0)?(dx-len):(dx+len);
}

//returns the longitudinal pos of obstacle vehicle, in relation to the ego vehicle
double circular_x(double ego_x, double obstacle_x){
	double dx = circular_dx(obstacle_x - ego_x);
	

	return ego_x + dx;
}


//returns the x,y longitudinal and lateral position of the corresponding id, in relation to the ego id
void get_relative_position(size_t ego_id, size_t obstacle_id, double *x, double *y){
	int ego_id_index = get_index(ego_id);
	assert(ego_id_index!= -1);

	int obstacle_id_index = get_index(obstacle_id);
	assert(obstacle_id_index!= -1);

	double ego_x, obstacle_x;
	

	ego_x = sim_env_global->x[ego_id_index];
	obstacle_x = sim_env_global->x[obstacle_id_index];

	*x = circular_x(ego_x,obstacle_x);
	*y = sim_env_global->y[obstacle_id_index];

}



//returns the longitudinal position of the corresponding id
double get_position_x(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	return sim_env_global->x[id_index];
	
}



//returns the longitudinal position of the corresponding id, in relation to the ego id
double get_relative_position_x(size_t ego_id, size_t obstacle_id){
	int ego_id_index = get_index(ego_id);
	assert(ego_id_index!= -1);

	int obstacle_id_index = get_index(obstacle_id);
	assert(obstacle_id_index!= -1);

	double ego_x, obstacle_x;
	

	ego_x = sim_env_global->x[ego_id_index];
	obstacle_x = sim_env_global->x[obstacle_id_index];

	return circular_x(ego_x,obstacle_x);
	


}


//returns the lateral position of the corresponding id
double get_position_y(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	return sim_env_global->y[id_index];
	
}

//returns the x,y longitudinal and lateral speed of the corresponding id to the assigned pointers
void get_speed(size_t id, double *vx, double *vy){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	*vx = sim_env_global->vx[id_index];
	*vy = sim_env_global->vy[id_index];
}

//returns the longitudinal speed of the corresponding id
double get_speed_x(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	return sim_env_global->vx[id_index];
	
}

//returns the lateral speed of the corresponding id
double get_speed_y(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	return sim_env_global->vy[id_index];
	
}


//returns the desired longitudinal speed of the corresponding id
double get_desired_speed(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);	

	return sim_env_global->vd[id_index];
}

//set a new desired longitudinal speed for the corresponding id
void set_desired_speed(size_t id, double des_speed){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	sim_env_global->vd[id_index] = des_speed; 
}

//returns the length and width of the corresponding id to the assigned pointers
void get_dimensions(size_t id, double *length, double *width){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	int ind = sim_env_global->class_id[id_index];
	*length = sim_env_global->l[ind];
	*width = sim_env_global->w[ind];
}

//returns the length of the corresponding id
double get_length(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	int ind = sim_env_global->class_id[id_index];
	return sim_env_global->l[ind];
	
}

//returns the width of the corresponding id
double get_width(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	int ind = sim_env_global->class_id[id_index];
	return sim_env_global->w[ind];
	
}
//returns the length and width of the corresponding type of vehicles to the assigned pointers
void get_type_dimensions(int type_id, double *length, double *width){

	*length = sim_env_global->l[type_id];
	*width = sim_env_global->w[type_id];

}


//returns the length of the corresponding type of vehicles
double get_type_length(int type_id){
	

	return sim_env_global->l[type_id];
	
}

//returns the width of the corresponding type of vehicles
double get_type_width(int type_id){
	
	return sim_env_global->w[type_id];
	
}

//returns the type id of the corresponding vehicle id. (the integer corresponds to the classes defined, e.g.: type_id=0 corresponds to the first type that was defined)
int get_type_id(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	return sim_env_global->class_id[id_index];
	
}

//returns an array of all vehicle ids inside the road network
size_t* get_ids(){
	assert(sim_env_global != NULL);

	return sim_env_global->ids;
}

//number of vehicles
int get_number_of_vehicles(){
	assert(sim_env_global != NULL);

	return sim_env_global->n;
}

//returns an array of all ego vehicle ids inside the road network
size_t* get_ego_ids(){
	assert(sim_env_global != NULL);

	return sim_env_global->ego_ids;
}

//number of ego vehicles
int get_number_of_ego_vehicles(){
	assert(sim_env_global != NULL);

	return sim_env_global->n_egos;
}


//check if a vehicle is ego or not
int is_ego(size_t id){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	int i, n = sim_env_global->ego_n_class_ids;
	int cid = sim_env_global->class_id[id_index];

	for (i=0;i<n;i++){
		if(cid==sim_env_global->ego_class_ids[i]){
			return 1;
		}
	}


	return 0;

}


//returns the road length (in meters)
double get_road_length(){
	assert(sim_env_global != NULL);

	return sim_env_global->roadlength_meters;
}


//returns the road width (in meters)
double get_road_width(){
	assert(sim_env_global != NULL);
	
	return sim_env_global->roadwidth_meters;
}

//returns the total number of timesteps
int get_number_of_timesteps(){
	assert(sim_env_global != NULL);

	return sim_env_global->all_timesteps;
}

//returns the current timestep
int get_current_timestep(){
	assert(sim_env_global != NULL);

	return sim_env_global->t;
}

//returns the timestep length (in seconds)
double get_timestep_T(){
	assert(sim_env_global != NULL);

	return sim_env_global->T;
}


//returns the inflow of vehicles in vehicles per hour
int get_inflow_vehs_per_hour(void){
	return sim_env_global->inflow;
}

//returns the number of decimals in double values
int get_double_decimals(void){
	return sim_env_global->double_decimals;
}

//returns the specified penetration rate specified. If no penetration rate is defined, it returns -1
double get_penetration_rate(void){
	return sim_env_global->penetration_rate;
}

//returns the seed
int get_seed(void){
	return sim_env_global->seed;
}


//apply control (acceleration) to the vehicle with the corresponding id
void apply_control(size_t id, double fx, double fy){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	

	sim_env_global->fx[id_index] = fx;
	sim_env_global->fy[id_index] = fy;

	sim_env_global->ux[id_index] = fx;
	sim_env_global->uy[id_index] = fy;

	sim_env_global->mode[id_index] = 0;
}


//apply control (acceleration) to the vehicle with the corresponding id, according to equations of motion
void apply_control_f(size_t id, double fx, double fy){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	
	sim_env_global->fx[id_index] = fx;
	sim_env_global->fy[id_index] = fy;

	sim_env_global->ux[id_index] = fx;
	sim_env_global->uy[id_index] = fy;

	sim_env_global->mode[id_index] = 1;
}


//directly control the position of the vehicle with the corresponding id
void move_vehicle(size_t id, double x, double y){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	
	sim_env_global->fx[id_index] = x;
	sim_env_global->fy[id_index] = y;

	sim_env_global->mode[id_index] = 2;
}

//directly control the position of the vehicle with the corresponding id based on jerk equations
void move_vehicle_jerk(size_t id, double x, double y){
	int id_index = get_index(id);
	assert(id_index!= -1);
	
	
	
	sim_env_global->fx[id_index] = x;
	sim_env_global->fy[id_index] = y;

	sim_env_global->mode[id_index] = 3;
}

int get_index(size_t id){
	assert(sim_env_global!=NULL);
	int i = 0;
	for(i=0;i<sim_env_global->n;i++){
		if(id==sim_env_global->ids[i]){
			return i;
		}
	}
	return -1;
}

//returns the x,y longitudinal and lateral acceleration of the corresponding id to the assigned pointers
void get_acceleration(size_t id, double *ux, double *uy){
	int id_index = get_index(id);
	assert(id_index!= -1);
	/*
	if (sim_env_global->mode[id_index] == 3){
		*ux = sim_env_global->ux[id_index];
		*uy = sim_env_global->uy[id_index];
		return;
	}*/
	if (sim_env_global->mode[id_index] != 2){
		*ux = sim_env_global->ux[id_index];
		*uy = sim_env_global->uy[id_index];
		return;
	}
	double vx,vy,dx,dy;
	dx = (sim_env_global->fx[id_index] - sim_env_global->x[id_index]);
	vx = 2*dx/get_timestep_T() - sim_env_global->vx[id_index];
	if (dx==0)
	{
		*ux = 0;
	}
	else
	{
		*ux = (pow(vx,2) - pow(sim_env_global->vx[id_index],2))/(2*dx);
	}
	
	
	dy = (sim_env_global->fy[id_index] - sim_env_global->y[id_index]);
	vy = 2*dy/get_timestep_T() - sim_env_global->vy[id_index];
	
	
	if (dy==0)
	{
		*uy = 0;
	}
	else
	{
		*uy = (pow(vy,2) - pow(sim_env_global->vy[id_index],2))/(2*dy);
	}

}

//returns the longitudinal acceleration of the corresponding id
double get_acceleration_x(size_t id){
	int id_index = get_index(id);

	assert(id_index!= -1);
	/*
	if (sim_env_global->mode[id_index] == 3){
		
		return sim_env_global->ux[id_index];
	}*/
	

	if (sim_env_global->mode[id_index] != 2){
		return sim_env_global->ux[id_index];
		
	}
	double vx, dx;
	dx = (sim_env_global->fx[id_index] - sim_env_global->x[id_index]);
	vx = 2*dx/get_timestep_T() - sim_env_global->vx[id_index];
	if (dx==0){
		return 0;
	}
	return (pow(vx,2) - pow(sim_env_global->vx[id_index],2))/(2*dx);
	
	
}

//returns the lateral acceleration of the corresponding id
double get_acceleration_y(size_t id){
	int id_index = get_index(id);

	assert(id_index!= -1);
	/*
	if (sim_env_global->mode[id_index] == 3){
		
		return sim_env_global->uy[id_index];
	}*/
	if (sim_env_global->mode[id_index] != 2){
		return sim_env_global->uy[id_index];
		
	}
	
	double vy, dy;
	dy = (sim_env_global->fy[id_index] - sim_env_global->y[id_index]);
	vy = 2*dy/get_timestep_T() - sim_env_global->vy[id_index];
	if (dy==0){
		return 0;
	}
	
	return (pow(vy,2) - pow(sim_env_global->vy[id_index],2))/(2*dy);
	
	
}
//internal API

void init_sim_struct(double road_length, double road_width, double T, int all_timesteps, int* ego_class_ids, int size_ego_classes, double* length, double* width, int size_classes, int double_decimals, int inflow, double penetration_rate, int seed, int circular){
	sim_env_global = (sim_env *)malloc(sizeof(sim_env));
	assert(sim_env_global!=NULL);
	sim_env_global->n = 0;
	sim_env_global->roadlength_meters = road_length;
	
	sim_env_global->roadwidth_meters = road_width;
	
	sim_env_global->T = T;
	sim_env_global->all_timesteps = all_timesteps;
	sim_env_global->ego_class_ids = (int *)malloc(size_ego_classes*sizeof(int));
	sim_env_global->ego_n_class_ids = size_ego_classes;
	sim_env_global->double_decimals = double_decimals;
	sim_env_global->inflow = inflow;
	sim_env_global->penetration_rate = penetration_rate;
	sim_env_global->seed = seed;
	sim_env_global->circular = circular;
	int i;
	for (i=0;i<size_ego_classes;i++){
		sim_env_global->ego_class_ids[i] = ego_class_ids[i];
	}

	sim_env_global->l = (double *)malloc(size_classes*sizeof(double));
	sim_env_global->w = (double *)malloc(size_classes*sizeof(double));

	for (i=0;i<size_classes;i++){
		sim_env_global->l[i] = length[i];
		sim_env_global->w[i] = width[i];
	}
}

void finalize_sim_struct(){
	
	free(sim_env_global->ego_class_ids);
	free(sim_env_global->l);
	free(sim_env_global->w);
	free(sim_env_global);
}

void update_sim_env(int t, int n, size_t* ids,int n_egos, size_t* ego_ids, int* class_id, double *v_d, double *pos_x, double *pos_y, double *v_x, double *v_y, double *u_x, double *u_y,
                         double *f_x, double *f_y, int* mode){
	
	sim_env_global->t = t;
    sim_env_global->n = n;
    sim_env_global->ids = ids;
	sim_env_global->class_id = class_id;
    sim_env_global->vd = v_d;
    sim_env_global->x = pos_x;
    sim_env_global->y = pos_y;
    sim_env_global->vx = v_x;
    sim_env_global->vy = v_y;
	sim_env_global->ux = u_x;
    sim_env_global->uy = u_y; 
    sim_env_global->fx = f_x;
    sim_env_global->fy = f_y;
    sim_env_global->mode = mode;

	sim_env_global->n_egos = n_egos;
	sim_env_global->ego_ids = ego_ids;
	
}

