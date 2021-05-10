#include <stdio.h>
//#include <sys/time.h>
#include "math.h"
#include <assert.h>
#include <string.h>

#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <stdlib.h>

#include "LaneFree_win.h"
#include "libLaneFreePlugin_Export.h"
#endif

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35

size_t ego_max_count=650, max_numsteps=40, obst_max_count=350, travel_steps=600*4, ego_count[1]={0}, time_lim1=3, time_lim2 = 8, sz2[1]={0}, data_offset=9;
double ***ego_history, speed_step_lim1=1, speed_step_lim2=3, speed_lim=25, run_time[1]={0}, road_length, road_width, safety1, safety2;
NumericalID *det_ids, n_det;
int *det_val, n_init=0, virtual_lanes = 4;
//#include "fda/fda_sdcb.c" //acceleration control
//#include "fda/fda.c" //acceleration control
//#include "fda/mpc.c"
//#include "fda/sort_func.c"

FILE *crash_fd = NULL, *orb_fd = NULL, *stats_fd = NULL;
char hist_fn[100]={0}, s[64];
size_t det_count[5]={0};
double **ev_video;

double drand48() {
	return (rand() / (RAND_MAX + 1.0));
}

void simulation_initialize(){

	srand(get_seed());

	FILE *ip_fd = NULL;
	if ((ip_fd = fopen("input2.txt","r"))==NULL)
		{
		printf("\nCouldn't read input.txt data file \n");
	}
	char buf[1024];
  	double dval;
	size_t ival, ival2;
	double speed_step;
	double penalty_input[50];
  while (fgets(buf, sizeof(buf), ip_fd)){ //to read from file
  //while (fgets(buf, sizeof(buf), stdin)){ //to read from standard input
    if (buf[0] == '\n')
      break;
    if (sscanf(buf, "\"speed_step\":%lf", &dval) == 1){
      speed_step = dval;
			fprintf(stderr, "speed_step: %f\n", speed_step);
		}
    if (sscanf(buf, "\"safety1\":%lf", &dval) == 1){
			safety1 = dval;
			fprintf(stderr, "safety1: %f\n", safety1);
		}
    if (sscanf(buf, "\"safety2\":%lf", &dval) == 1){
      safety2 = dval;
			fprintf(stderr, "safety2: %f\n", safety2);
		}
    if (sscanf(buf, "\"penalty(%zu)\":%lf",&ival, &dval) == 1){
      penalty_input[ival] = dval;
			fprintf(stderr, "pentalty:%zu, %f\n", ival, penalty_input[ival]);
		}
		if (sscanf(buf, "\"n_veh\":%zu", &ival) == 1){
			n_init = ival;
			fprintf(stderr, "Number of vehicles: %d\n", n_init);
		}
	}

	fclose(ip_fd);
	printf("After this\n");
	char fn[100]={0}, *buffer, *linestatus=NULL, *token=NULL, delimiter[2] = "\t";
	assert(buffer=(char*)calloc(sizeof(char),12*travel_steps+1)); //line length for (11 character numbers + space)*steps + \n
	assert(ev_video=(double**)calloc(sizeof(double*),ego_max_count*data_offset));
	for (size_t i = 0; i < ego_max_count*data_offset; i++) {
		assert(ev_video[i]=(double*)calloc(sizeof(double),travel_steps));
	}
  sprintf(fn, "50_hist.log");
	int n_init=50;
	//sprintf(fn, "egos_hist_3.bin");
	FILE *fptr = NULL;
	if ((fptr = fopen(fn,"r"))==NULL){
	 printf("\nCouldn't open data file \n");
	 exit(1);
	}
	size_t sz[2]={0};
  for(sz[0]=0; (linestatus=fgets(buffer,12*travel_steps+1,fptr))!=NULL; sz[0]++){
    int len=strlen(buffer);
    buffer[len-1]='\0'; //removing \n EOL
    token=strtok(buffer,delimiter);
    for(sz[1]=0; token!=NULL; sz[1]++){
			ev_video[sz[0]][sz[1]]=atof(token);
    	token=strtok(NULL,delimiter);
    }
  }
  fclose(fptr);

	//int n_init = 4*10; // To insert vehicles
	char veh_name[40];
	//route_id and type_id should be defined in the scenario we are running
	char route_id[20]="route0";
	road_length = get_edge_length(0);
	road_width = get_edge_width(0);
	double x_incr = (road_length-3.0)/(ceil((double)n_init/virtual_lanes)), y_incr=road_width/virtual_lanes, virtual_lane_width = road_width/virtual_lanes, roadbound_safety=0.2, interlane_safety=0.2;
	double x_val=x_incr/2, y_val=y_incr/2, vx_val=0;
	char type_id[8][2]={"1","2","3","4","5","6","7","8"};
	NumericalID v_id;
	int k=0;
	double temp_x, temp_y;
	for(int i=0;i<n_init;i++){
		k=k%8;
		sprintf(veh_name, "%s_plugin_%d", type_id[k], i);
		v_id = insert_new_vehicle(veh_name, route_id, type_id[k], x_val + (drand48())*0.5*x_incr - 0.25*x_incr, y_val + (drand48())*0.05 - 0.025, vx_val,0);
		y_val += y_incr;
		if(i%virtual_lanes==(virtual_lanes-1)){
			x_val += x_incr;
			vx_val =0;
			y_val = y_incr/2;
			//fprintf(stderr, "\n");
		}
		k++;
	}
	det_ids = get_detectors_ids();
	n_det = get_detectors_size();
	
}

void simulation_step() {
	printf("Timestep: %d\n", get_current_time_step());
	NumericalID* ego_ids = get_lane_free_ids();

	printf("got ids\n");
	NumericalID n_egos = get_lane_free_ids_size();//, n_veh = get_all_ids_size();
	printf("num of vehicles entered: %lld\n", n_egos);
	for (size_t i = 0; i < n_egos; i++) {
		fprintf(stderr, "%lld\n", ego_ids[i]);
		apply_acceleration((NumericalID)ev_video[9*(i+1)-2][0], ev_video[9*i+5][sz2[0]], ev_video[9*i+6][sz2[0]]);
	}
	//fprintf(stderr, "Position: %f\n", get_position_x((NumericalID)ev_video[7][0]));
}

void simulation_finalize(){

}


void event_vehicle_enter(NumericalID veh_id){
	int min_speed = 25, max_speed = 35;
	set_desired_speed(veh_id, (double)(rand() % (int)(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED));
	fprintf(stderr, "%lld, %f, %f, %f, %f, %f\n", veh_id, get_veh_length(veh_id), get_veh_width(veh_id), get_desired_speed(veh_id), get_position_x(veh_id), get_position_y(veh_id));
	set_circular_movement(veh_id, true);
}

void event_vehicle_exit(NumericalID veh_id){
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2){
	char name_buffer1[64], name_buffer2[64];
	char *vname1 = get_vehicle_name(veh_id1);
	snprintf(name_buffer1, 64,"%s", vname1);
	char *vname2 = get_vehicle_name(veh_id2);
	snprintf(name_buffer2, 64,"%s", vname2);
	//fprintf(stderr, "Collision between %s (%.2f, %.2f) and %s (%.2f, %.2f) at timestep: %d, and time: %.1f.\n",get_vehicle_name(veh_id1), get_position_x(veh_id1), get_position_y(veh_id1), get_vehicle_name(veh_id2), get_position_x(veh_id2), get_position_y(veh_id2), get_current_time_step(), get_current_time_step()*get_time_step_length());
	fprintf(stderr, "Collision between %s (%.2f, %.2f) and %s (%.2f, %.2f) at timestep: %d, and time: %.1f.\n",name_buffer1, get_position_x(veh_id1), get_position_y(veh_id1), name_buffer2, get_position_x(veh_id2), get_position_y(veh_id2), get_current_time_step(), get_current_time_step()*get_time_step_length());
}
void event_vehicle_out_of_bounds(NumericalID veh_id) {
	char* vname1 = get_vehicle_name(veh_id);
	fprintf(stderr, "Vehicle %s is out of bounds at time %.2f, at pos:%f,%f.\n", vname1, get_current_time_step() * get_time_step_length(), get_position_x(veh_id),get_position_y(veh_id));
}
