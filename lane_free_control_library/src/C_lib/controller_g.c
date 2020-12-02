#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include <math.h>
#include "controller_g.h"
#include "controller_api.h"

#define NUM_THREADS 6



//Executed once, before simulation begins
void sim_initialize()
{
    int precision = get_double_decimals();
    int inflow_vehs_per_hour = get_inflow_vehs_per_hour();
    double penetration_rate = get_penetration_rate();
    int seed = get_seed();
    srand(seed);
        
    //printf("Decimal points: %d\n",precision);
    //printf("Flow (vehicles per hour): %d\n",inflow_vehs_per_hour);
    //printf("Penetration rate: %f\n",penetration_rate);
    

	
}

//Executed once, before simulation begins. You may overwrite the initial placement of vehicles
void sim_initialize_vehicles(int n, int* type, double* x, double* y, double* vx){	
    
    
    /*
    //dummy placement
	double road_length = get_road_length();
	double road_width = get_road_width();
    double vehicle_length, vehicle_width;
    int type_i;
    double place_every = road_length/n;
    double placement_x = place_every/2;
    double upper_y,lower_y;

	for(int i=0;i<n;i++){
        type_i = type[i];
        vehicle_length = get_type_length(type_i);
        vehicle_width = get_type_width(type_i);
		x[i] = placement_x;
        upper_y = road_width - vehicle_width/2;
        lower_y = vehicle_width/2;

		y[i] =  (drand48())*(upper_y - lower_y) + lower_y;
		vx[i] = 0;

        placement_x += place_every;
		
	}
    */
    
}

//Executed every time a new vehicle with id enter_id enters the road network
void sim_vehicle_enter(size_t enter_id)
{
    //printf("Vehicle %d just entered, with speed %f.\n",(int)enter_id,get_speed_x(enter_id));
}

//Executed every time a vehicle with id exit_id exits the road network
void sim_vehicle_exit(size_t exit_id)
{
    //printf("Vehicle %d is exiting.\n",(int)exit_id);
}

//Executed every time two vehicles with ids crash_id_1, crash_id_2 collide
void sim_vehicles_collision(size_t crash_id_1, size_t crash_id_2)
{
    //printf("Collision between %d and %d.\n",(int)crash_id_1,(int)crash_id_2);
}

//Executed every time a vehicle with id out_bound_id is located outside the road boundary
void sim_vehicle_out_of_bound(size_t out_bound_id)
{
    //printf("Vehicle %d out of bounds.\n",(int)out_bound_id);
}
//You may use a predefined set of functions to retrieve inormation regarding all vehicles, and also the ability to apply control
void sim_run()
{   
    //printf("%lf\n",v1);
    
    
    
    /*
    int test[20];
    #pragma omp parallel for num_threads(NUM_THREADS)
    for (size_t i = 0; i < 20; i++) {
        for(int j =0;j<10000;j++){
            test[i] += test[i]+i*j;
        }
        printf("Thread number: %d out of %d threads\n", omp_get_thread_num(), omp_get_num_threads());
    }*/
    
    double road_width = get_road_width();
    // printf("road width: %A\n",10.2);
    
    int i, n_egos = get_number_of_ego_vehicles();
    if (n_egos==0){
        return;
    }
    size_t *ids = get_ego_ids();

    /*
    for(i=0;i<n_egos;i++)
    {
        printf("%zu\n",ids[i]);
    }
    */
    /*
    printf("For vehicle:%d\n",(int)ids[0]);
    double r_posx, r_posy;
    for(i=1;i<n_egos;i++){
        get_relative_position(ids[0],ids[i],&r_posx,&r_posy);
        printf("Obstacle vehicle %d, at relative position (%f,%f)\n",(int)ids[i],r_posx,r_posy);
    }
    */
    size_t id;
    double posx, posy, new_x, new_y;
    id = ids[0];

    double vx, vy;
    double dt = get_timestep_T();
    id = ids[0];

    vx = get_speed_x(id);
    vy = get_speed_y(id);

    //printf("vehicle %d: vx:%f, vy:%f\n", id, vx, vy);

    id = ids[0];

    double vd;

    //apply control
    double  ux_get, uy_get, jx,jy,dx, dy, jx_est;
    
    // int id_change_des_speed = 2;
    for (i=0;i<n_egos;i++)
    {
        // if(i==id_change_des_speed){
        //     set_desired_speed(ids[i],20);
        // }
        posx = get_position_x(ids[i]);
        posy = get_position_y(ids[i]);

        vx = get_speed_x(ids[i]);
        vy = get_speed_y(ids[i]);
        ux_get = get_acceleration_x(ids[i]);
        uy_get = get_acceleration_y(ids[i]);
        
        
        //printf("vehicle %zu: x:%f, y:%f, vx:%f, vy:%f\n", ids[i], posx, posy, vx, vy);
        
        
        vd = get_desired_speed(ids[i]);
        if(vx<vd && ux_get<1){
            jx = 0.1;
        }
        else{
            jx = -0.1;
        }
        
        jy = 0;

        //apply_control_f(ids[i], ux, uy);
        
        

        dx =  vx*dt + 0.5*ux_get*pow(dt,2) + jx*pow(dt,3)/6;
        dy =  vy*dt + 0.5*uy_get*pow(dt,2) + jy*pow(dt,3)/6;
        
        
        
        new_x = dx + posx;
        new_y = dy + posy;
        // printf("vehicle %zu: x:%.15f, dx:%.15f, y:%.15f, vx:%.15f, vy:%.15f, ux:%.15f, uy:%.15f \t", ids[i], posx, dx, posy, vx, vy, ux_get, uy_get);
        
        move_vehicle_jerk(ids[i], new_x, new_y);

        
        vx = vx + ux_get*dt+0.5*jx*pow(dt,2);
        vy = vy + uy_get*dt+0.5*jy*pow(dt,2);

        ux_get = ux_get + jx*dt;
        uy_get= uy_get + jy*dt;
        // printf("next state: vx:%.15f, vy:%.15f, ux:%.15f, uy:%.15f\n",vx,vy,ux_get, uy_get);

        

        //printf("longitudinal: actual:%f, get:%f, get_p:%f\n",ux,ux_get,ux_get_d);
        //printf("lateral: actual:%f, get:%f, get_p:%f\n",uy,uy_get,uy_get_d);
    
    }
    
    
    /*
    id = 1;

    //directly move vehicle    
    double x_new, y_new;

    if(is_ego(id))
    {
        x_new = 400;
        y_new = 5.2;

        id = 1;
        move_vehicle(id, x_new, y_new);
    }

    */
    

}


//Executed once, after the last simulation timestep
void sim_finalize()
{
    printf("Finalize\n");
}