#ifdef CONTROLLER_H
#define EXTERN_C /* nothing */
#else
#define EXTERN_C extern
#endif /* DEFINE_VARIABLES */

//NumericalID is used as a data type for ids.
typedef long long int NumericalID;
typedef enum {FCA_0, FCA_1, FCA_2, FCA_3} fca_method_t;


typedef struct {
	int j;
	double mag;
	double fcax, fcay;
} nbor_t;



typedef	struct {
	struct {
		double position;
		double distance;
		double velocity;
		long double bound;
		int leader;
		double degree;
	}x;
	struct {
		double up, degree_up;
		double dn, degree_dn;
		int up_j, dn_j;
		long double bound_up, bound_dn;
	}y;
}walls_t;

typedef struct {
	double alpha;
	double frame_timegap;
	int ftsx_disabled;
	int fca_nbors, fca_nbors_nudge;
	int fca_sat;
	double duet, duet_vd, duet_vd_slow, duet_dy, duet_v0;

	int reg;
	int fca_max, four_lane_init;
	int barrier;
	double single_lane_space, single_lane_front_vd;
	int single_lane;
	int dynamic_y_walls, dynamic_x_walls;
	double vd_throttling_horizon;
	int warmup, vd_throttling;
	double influence_radius_meters;
	double fwd_force_max_x, fwd_force_max_y;
	double bwd_force_max_x, bwd_force_max_y;

	fca_method_t fca_method;
	double time_gap_x, time_gap_y;
	int zero_controls, zero_initial_speed, lanedrop, virtual_lanes;
	int n, K;
	double uxmax_hard, uxmin_hard, uymax_hard;
	double T, vd_meters_per_sec_lo, vd_meters_per_sec_hi;
	double vd_meters_per_sec_lo_truck, vd_meters_per_sec_hi_truck;
	double roadlen_meters, roadwid_meters;
	double ftsx_zeta, ftsx_hi, ftsx_lo;
	double ftsy_zeta, ftsy_hi;
	int *truck;
	double **x, **y, **vx, **vy, **ux, **uy, *w, *l, *vd;
	double wall_y_up, wall_y_dn, wall_y_up_j, wall_y_dn_j;
	walls_t walls;
	double **fx, **fy;
	int **crash;
	double *flow;
	double *vdx_effective, *vdy_effective;
	double coeff_fcax, coeff_fcay, fcax_zeta, fcay_zeta;
	double coeff_fmdl;
	double safety_level_x, safety_level_y;
	
	double ***degree_x;
	double ***degree_y;
	int crashes, crashes_on_leaders;
	double *class_w, *class_l;
	int ring_network;
	double vd_alpha;
}sim_t;


void determine_forces(sim_t* sim, NumericalID edge_id, int i, NumericalID* vehs_array, int n, double* fx, double* fy);
void regulate_forces(sim_t* sim, NumericalID edge_id, NumericalID veh_id, double* fx, double* fy);
void determine_controls(sim_t* sim, double* fx, double* fy);


void sim_configure(sim_t *sim);


EXTERN_C nbor_t* nbors;
EXTERN_C nbor_t* nbors_nudge;
