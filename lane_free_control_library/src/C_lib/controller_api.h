

//returns the x,y longitudinal and lateral position of the corresponding id to the assigned pointers 
void get_position(size_t id, double *x, double *y);


//returns the x,y longitudinal and lateral position of the corresponding id, in relation to the ego id
void get_relative_position(size_t ego_id, size_t obstacle_id, double *x, double *y);


//returns the longitudinal position of the corresponding id
double get_position_x(size_t id);


//returns the longitudinal position of the corresponding id, in relation to the ego id
double get_relative_position_x(size_t ego_id, size_t obstacle_id);


//returns the lateral position of the corresponding id
double get_position_y(size_t id);


//returns the x,y longitudinal and lateral speed of the corresponding id to the assigned pointers
void get_speed(size_t id, double *x, double *y);


//returns the longitudinal speed of the corresponding id
double get_speed_x(size_t id);


//returns the lateral speed of the corresponding id
double get_speed_y(size_t id);


//returns the x,y longitudinal and lateral acceleration of the corresponding id to the assigned pointers
void get_acceleration(size_t id, double *ux, double *uy);


//returns the longitudinal acceleration of the corresponding id
double get_acceleration_x(size_t id);


//returns the lateral acceleration of the corresponding id
double get_acceleration_y(size_t id);

//returns the desired longitudinal speed of the corresponding id
double get_desired_speed(size_t id);

//set a new desired longitudinal speed for the corresponding id
void set_desired_speed(size_t id, double des_speed);

//returns the length and width of the corresponding id to the assigned pointers
void get_dimensions(size_t id, double *length, double *width);


//returns the length of the corresponding id
double get_length(size_t id);


//returns the width of the corresponding id
double get_width(size_t id);


//returns the length and width of the corresponding type of vehicles to the assigned pointers
void get_type_dimensions(int type_id, double *length, double *width);


//returns the length of the corresponding type of vehicles
double get_type_length(int type_id);


//returns the width of the corresponding type of vehicles
double get_type_width(int type_id);


//returns the type id of the corresponding vehicle id. (the integer corresponds to the classes defined, e.g.: type_id=0 corresponds to the first type that was defined)
int get_type_id(size_t id);


//returns an array of all vehicle ids inside the road network
size_t* get_ids(void);

//number of vehicles
int get_number_of_vehicles(void);


//returns an array of all ego vehicle ids inside the road network
size_t* get_ego_ids(void);

//number of ego vehicles
int get_number_of_ego_vehicles(void);

//check if a vehicle is ego or not, returns 1 if vehicle is ego, else 0
int is_ego(size_t id);


//returns the road length (in meters)
double get_road_length(void);


//returns the road width (in meters)
double get_road_width(void);


//returns the total number of timesteps
int get_number_of_timesteps(void);


//returns the current timestep
int get_current_timestep(void);


//returns the timestep length (in seconds)
double get_timestep_T(void);


//returns the inflow of vehicles in vehicles per hour
int get_inflow_vehs_per_hour(void);


//returns the number of decimals in double values
int get_double_decimals(void);


//returns the specified penetration rate specified. If no penetration rate is defined, it returns -1
double get_penetration_rate(void);

//returns the seed
int get_seed(void);


//apply control (acceleration) to the vehicle with the corresponding id
void apply_control(size_t id, double ux, double uy);


//apply control (acceleration) to the vehicle with the corresponding id, according to equations of motion
void apply_control_f(size_t id, double ux, double uy);


//directly control the position of the vehicle with the corresponding id
void move_vehicle(size_t id, double x, double y);


//directly control the position of the vehicle with the corresponding id based on jerk equations
void move_vehicle_jerk(size_t id, double x, double y);


//simulation information structure
typedef struct {
	
	int n, n_egos, t, all_timesteps;
	double T;
	
	double roadlength_meters, roadwidth_meters;
	
	size_t *ids;
	size_t *lf_ids;
	double *fx, *fy;
	int *mode;	
	
	int ego_n_class_ids;
	int double_decimals;
	int inflow;
	int seed;
	double penetration_rate;
	int circular;
}sim_env;


typedef struct 
{
	double x,y,vx,vy,ux,uy,w,l,vd;
	long long int id;
	int class_id;
	int is_lanefree;


}sim_vehicle;
//internal API




void init_sim_struct(double road_length, double road_width, double T, int all_timesteps, int* ego_class_ids, int size_ego_classes, double* length, double* width, int size_classes, int double_decimals, int inflow, double penetration_rate, int seed, int circular);

void finalize_sim_struct(void);


void update_sim_env(int t, int n, size_t* ids, int n_egos, size_t* ego_ids, int* class_id, double *v_d, double *pos_x, double *pos_y, double *v_x, double *v_y, double *u_x, double *u_y,
                         double *f_x, double *f_y, int* mode);