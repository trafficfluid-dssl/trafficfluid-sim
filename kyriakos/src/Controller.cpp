#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <omp.h>

#define PRINT_XY_DEGREES 0
#define PARALLEL_RUN 1


#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree_win.h>
#include "libLaneFreePlugin_Export.h"
#endif


#define CONTROLLER_H
#include "Controller.h"


#define BUS_RATIO 0
double LANEWIDTH;
#define VD_VARIANCE 2.0
#define XBUFFER 2.0

#define SAMPLE_UNIFORM(min, max) ((double)min + ((double)random()/RAND_MAX)*(max - min))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))




#define WALL_DN(point, safety, v, yi, wi) U_lemma3( -(safety)*v, MAX(0, yi-0.5*wi - (point)), -sim->uymax_hard)
#define WALL_UP(point, safety, v, yi, wi) U_lemma3( +(safety)*v, MAX(0, (point) - (yi+0.5*wi)), -sim->uymax_hard)

static double
circular_dx(sim_t* sim, double dx, int is_circular) {
    if (!is_circular) {
        return dx;
    }
    double len = sim->roadlen_meters;
    if (fabs(dx) <= 0.5 * len)
        return dx;
    else
        return (dx >= 0) ? (dx - len) : (dx + len);
}


static double
ftsx(sim_t *sim, double vx, double vd) {
    //return smooth_max(sim->ftsx_lo, sim->ftsx_hi*(1 - exp(-sim->ftsx_zeta*(sim->vdx_effective[i]-sim->vx[i][t]))), 1.0);
    return erfc(vx-vd) - 1;
}

static double
ftsy(sim_t *sim, double vy) {
    //return sim->ftsy_hi*(2 / (1 + exp(-sim->ftsy_zeta*(sim->vdy_effective[i] - sim->vy[i][t]))) - 1);
    return erfc(vy) - 1;
}

static double
fmdl(sim_t *sim, NumericalID veh_id, NumericalID edge_id) {
    double vd_range = (sim->vd_meters_per_sec_hi - sim->vd_meters_per_sec_lo);
    double point_in_range = 0.5;
    double spread_factor = 1.0;
    double gravity_point;
    if (sim->virtual_lanes) {
        point_in_range = (get_desired_speed(veh_id) - sim->vd_meters_per_sec_lo)/vd_range;
    }

	double width = get_veh_width(veh_id), yv = get_position_y(veh_id);
    gravity_point = width/2 + (0 + point_in_range)*(get_edge_width(edge_id) - width)*spread_factor;
    return sim->coeff_fmdl*(2 / (1 + exp(-sim->ftsy_zeta*(gravity_point - yv))) - 1);
}

static double
U_lemma3(double v, double d, double ubar) {
    double nom, den; 
    double T = get_time_step_length();
    double ret;

    nom = T*ubar- 2*v + sqrt(pow(T,2)*pow(ubar,2)-(4*T*ubar*v)+8*ubar*(T*v-d));
    den = 2*T;

    if (fpclassify((ret = nom/den)) == FP_NAN) {
        ret = ubar;
    }

    return ret;
}

static double
G(double t, double warmup, double length, double cooldown) {
    return fmax(0, fmin(fmin(1. , t/warmup), 1. - (t-warmup-length)/cooldown));
}

static double
H(double t, double warmup, double length, double cooldown) {
    return G(t+warmup+length, warmup, 2*length, cooldown);
}

static long double
fca_mag(sim_t *sim, double dx, double dy, double vxi, double approaching_vx, double approaching_vy, double len, double wid)
{
    switch(sim->fca_method) 
    {
        case FCA_0:
        {
            double magx = pow(dx/len, 2);
            double magy = pow(dy/wid, 4);
            return (1.0/(1.0 + magx + magy));
        }

        case FCA_1:
        {
            double Lx = vxi*sim->time_gap_x;
            double Dx = 0.5*pow(approaching_vx, 2)/sim->coeff_fcax + len;
            double magx = H(dx, 0.5*Lx, 0.5*Lx + Dx + 2.5, 0.5*Lx);

            double Ly = approaching_vy*sim->time_gap_y + 0.5;
            double Dy = wid;
            double magy = pow(H(dy, Ly, Dy + 0.5, Ly), 1);

            //return (fmin(magx, magy));
            return pow(magx * magy, 1);
        }

        default:
            assert(!"unsupported fca_mag method");
    }
    return 0;
}

static double
fca(sim_t *sim, NumericalID veh_i, NumericalID veh_j, double dx_i_to_j, double dy_i_to_j, double *fcax, double *fcay) {
    double mag, ang;
	double li = get_veh_length(veh_i), lj = get_veh_length(veh_j);
	double wi = get_veh_width(veh_i), wj = get_veh_width(veh_j);
    double len = 0.5*(li + lj);
    double wid = 0.5*(wi + wj);

    *fcax = *fcay = 0;

    double x, x0, y, y0, approaching_speed_x, approaching_speed_y;
    x0 = get_position_x(veh_j);  
    y0 = get_position_y(veh_j);
    x = get_position_x(veh_i);
    y = get_position_y(veh_i);

	double vxi = get_speed_x(veh_i), vxj = get_speed_x(veh_j);
	double vyi = get_speed_y(veh_i), vyj = get_speed_y(veh_j);

    if (dx_i_to_j >= 0) {
        // i is approaching j from behind
        // mag depends on where i is located within j's aura
        approaching_speed_x = MAX(0, vxi - (1. - sim->safety_level_x)* vxj);
        approaching_speed_y = fabs(vyi - vyj);
    } else {
        // j is approaching i from behind
        // mag depends on where i is located within j's aura
        approaching_speed_x = vxj;
        approaching_speed_y = fabs(vyj - vyj);
    }

    mag = fca_mag(sim,
                -get_relative_distance_x(veh_i, veh_j), y-y0, 
                vxi,
                approaching_speed_x, approaching_speed_y, len, wid);
    
    ang = atan2(dy_i_to_j, dx_i_to_j);
    *fcax = -cos(ang) * mag;
    *fcay = -sin(ang) * mag;

    return mag;
}


static void
walls_init(sim_t *sim, NumericalID edge_id, NumericalID veh_id, walls_t *walls) {
	double roadlen_meters = get_edge_length(edge_id), roadwid_meters = get_edge_width(edge_id);
	double T = get_time_step_length();
	double vx = get_speed_x(veh_id), vy = get_speed_y(veh_id), vd = get_desired_speed(veh_id);
	walls->x.position = fmod(get_position_x(veh_id) + 0.5* roadlen_meters, roadlen_meters);
    walls->x.distance = 0.5* roadlen_meters;
    walls->x.velocity = vd;
    walls->x.leader = -1;
    walls->x.bound = (vd- vx) /T;
    walls->x.degree = 0;

    walls->y.dn = 0;
    walls->y.up = roadwid_meters;

    walls->y.up_j = -1;
    walls->y.dn_j = -1;

	double yi = get_position_y(veh_id), wi = get_veh_width(veh_id);
    walls->y.bound_dn = WALL_DN(0, sim->safety_level_y, vy, yi, wi);
    walls->y.bound_up = WALL_UP(roadwid_meters, sim->safety_level_y, vy, yi, wi);

    walls->y.degree_up = 0;
    walls->y.degree_dn = 0;
}

static void
walls_update(sim_t *sim, NumericalID veh_i, NumericalID veh_j, double fcax, double fcay, walls_t *walls) {
    if (veh_i == veh_j)
        return;
    
    // clip forces within [-1, 1]
    fcax = MIN(1.0, MAX(-1.0, fcax));
 
    // degree_x in [0, 1]: degree to which j is a front obstacle
    double degree_x = pow(fabs(MIN(fcax, 0)), 0.25);
	
	double vxi = get_speed_x(veh_i), vxj = get_speed_x(veh_j);
	double vdi = get_desired_speed(veh_i);

	double T = get_time_step_length();

	double lj = get_veh_length(veh_j), xj = get_position_x(veh_j);

    // determine front bound
	double dx = get_relative_distance_x(veh_i, veh_j) - (get_veh_length(veh_i) + get_veh_length(veh_j))/2;//circular_dx(sim, sim->x[j][t]-0.5*sim->l[j] - (sim->x[i][t]+0.5*sim->l[i]));
    double vs = MIN(dx/sim->frame_timegap, vxj - 0.1);
    double uxbound = ((1.-degree_x)*(vdi-vxi) + degree_x*(vs - vxi))/(1*T);

    if (degree_x > walls->x.degree) {
        walls->x.bound = uxbound;
        walls->x.leader = veh_j; //this is not the index as before. However it is not used anywhere, thus it should be ok
        walls->x.distance = dx;
        walls->x.position = xj -0.5* lj;
        walls->x.velocity = vxj;
        walls->x.degree = degree_x;
    }

#if (PRINT_XY_DEGREES == 1)
    sim->degree_x[i][t][j] = degree_x;
    sim->degree_y[i][t][j] = fabs(fcay);
#endif
    return;
}


int cmpnbors(const void *n1ptr, const void *n2ptr) {
    nbor_t *n1 = (nbor_t*)n1ptr;
    nbor_t *n2 = (nbor_t*)n2ptr;

    if (n1->mag == n2->mag)
        return 0;
    if (n1->mag < n2->mag)
        return -1;
    
    return +1;
}

void
//determine_forces(sim_t *sim, double T, double roadlen_meters, double roadwid_meters, int i, int n, double* x, double*y, double* vx, double* vy, double* vd, double* l, double* w, int is_circular, double* fx, double* fy) {
 determine_forces(sim_t* sim, NumericalID edge_id, int i, NumericalID* vehs_array, int n, double* fx, double* fy){
    int j;
    double fcax = 0, fcay = 0;

	//TODO: this is not efficient, do it like the internal array pointer structure (use global array pointers), also remove the free() at the end once we are done!
    nbor_t* nbors = (nbor_t*)malloc(n*sizeof(nbor_t));
    nbor_t* nbors_nudge = (nbor_t*)malloc(n * sizeof(nbor_t));
    //nbor_t nbors[425];
    //nbor_t nbors_nudge[425];
    memset(nbors, 0, sizeof(nbors));
    memset(nbors_nudge, 0, sizeof(nbors_nudge));

    
    walls_init(sim, edge_id, vehs_array[i], &(sim->walls));
    
    /* initalize forces to zero */
	
	double fxi = 0, fyi = 0;
	double half_roadlen_meters = get_edge_length(edge_id) / 2;    
    /* obstacle-related forces */
    double fcax_sum = 0, fcay_sum =0;
    int nbors_added = 0, nbors_added_nudge = 0;
	NumericalID veh_id = vehs_array[i];
    
	//one for from i until front_end, where front_end is n-1, and when it reaches n-1 it resets to zero. we break when either the ifluence_radius_meters is exceeded or we have relative distance >roadlength/2
	double dx_i_to_j, dy_i_to_j, fcamag;


	//Downstream vehicles
    
    for (j=(i+1); ; j ++) {
        if (j > n - 1) {
            j = 0;
        }    
		if (j == i)
			break;
        
        dx_i_to_j = get_relative_distance_x(veh_id, vehs_array[j]);
        dy_i_to_j = get_relative_distance_y(veh_id, vehs_array[j]);
        
        fcamag = 0;
        //printf("Vehicle id: %lld\n", vehs_array[j]);
        
        
        if ((dx_i_to_j) > MIN(sim->influence_radius_meters,half_roadlen_meters) || dx_i_to_j < 0)
            break;
        
        fcamag = fca(sim, vehs_array[i], vehs_array[j], dx_i_to_j, dy_i_to_j, &fcax, &fcay);


        if (fcax < 0) {
            nbors[nbors_added].j = j;
            nbors[nbors_added].mag = -fcamag;
            nbors[nbors_added].fcax = fcax;
            nbors[nbors_added].fcay = fcay;
            nbors_added++;
        }
        
        if (fcax > 0) {
            nbors_nudge[nbors_added_nudge].j = j;
            nbors_nudge[nbors_added_nudge].mag = -fcamag;
            nbors_nudge[nbors_added_nudge].fcax = fcax;
            nbors_nudge[nbors_added_nudge].fcay = fcay;
            nbors_added_nudge++;
        }
    }

   
	//Upstream vehicles
	for (j = i - 1; ; j --) {
        if (j < 0) {
            j = n - 1;
        }
		if (j == i)
			break;
		dx_i_to_j = get_relative_distance_x(veh_id, vehs_array[j]);
		dy_i_to_j = get_relative_distance_y(veh_id, vehs_array[j]);
		fcamag = 0;



		if ((-dx_i_to_j) > MIN(sim->influence_radius_meters, half_roadlen_meters) || dx_i_to_j > 0)
			break;

		fcamag = fca(sim, vehs_array[i], vehs_array[j], dx_i_to_j, dy_i_to_j, &fcax, &fcay);


		if (fcax < 0) {
			nbors[nbors_added].j = j;
			nbors[nbors_added].mag = -fcamag;
			nbors[nbors_added].fcax = fcax;
			nbors[nbors_added].fcay = fcay;
			nbors_added++;
		}

		if (fcax > 0) {
			nbors_nudge[nbors_added_nudge].j = j;
			nbors_nudge[nbors_added_nudge].mag = -fcamag;
			nbors_nudge[nbors_added_nudge].fcax = fcax;
			nbors_nudge[nbors_added_nudge].fcay = fcay;
			nbors_added_nudge++;
		}
	}
    // sort neighbors and compute fcax_sum from top sim->nbors
    if (sim->fca_nbors) {
        int x;
        qsort(nbors, nbors_added, sizeof(nbor_t), cmpnbors);
        fcax_sum = fcay_sum = 0;
        for (x=0; x < MIN(sim->fca_nbors, nbors_added); x++) {
            fcax_sum += nbors[x].fcax;
            fcay_sum += nbors[x].fcay;

            walls_update(sim, vehs_array[i], vehs_array[nbors[x].j], nbors[x].fcax, nbors[x].fcay, &(sim->walls));
        }
    }

    if (sim->fca_nbors_nudge) {
        int x;
        qsort(nbors_nudge, nbors_added_nudge, sizeof(nbor_t), cmpnbors);
        for (x=0; x < MIN(sim->fca_nbors_nudge, nbors_added_nudge); x++) {
            fcax_sum += nbors_nudge[x].fcax * sim->fwd_force_max_x;
            fcay_sum += nbors_nudge[x].fcay * sim->fwd_force_max_y;
        }
    }
    
    if (sim->fca_sat) {
        fcax_sum = MAX(-1, MIN(1, fcax_sum));
        fcay_sum = MAX(-1, MIN(1, fcay_sum));
    }

    fxi += sim->coeff_fcax * fcax_sum;
    fyi += sim->coeff_fcay * fcay_sum;

	double vxi = get_speed_x(veh_id), vyi = get_speed_y(veh_id), vdi = get_desired_speed(veh_id);
    /* target-speed related forces */
    if (!sim->ftsx_disabled)
        fxi += ftsx(sim, vxi, vdi);
    fyi += ftsy(sim, vyi);
    fyi += fmdl(sim, veh_id, edge_id);

    sim->wall_y_up = sim->walls.y.up;
    sim->wall_y_dn = sim->walls.y.dn;
    sim->wall_y_up_j = sim->walls.y.up_j;
    sim->wall_y_dn_j = sim->walls.y.dn_j;


	*fx = fxi;
	*fy = fyi;

	free(nbors);
	free(nbors_nudge);
}

void
regulate_forces(sim_t *sim, NumericalID edge_id, NumericalID veh_id, double* fx, double* fy) {
    /* y wall */
	double fxi = *fx, fyi = *fy;
    if (sim->dynamic_y_walls) {
        double uy_max = sim->walls.y.bound_up;
        double uy_min = sim->walls.y.bound_dn;
        if (uy_max >= -uy_min) {
			fyi = MAX(fyi, -uy_min);
			fyi = MIN(fyi, +uy_max);
        }
    }

	double vx = get_speed_x(veh_id), vd = get_desired_speed(veh_id), vy = get_speed_y(veh_id);
	double yi = get_position_y(veh_id), wi = get_veh_width(veh_id);
	double roadwid_meters = get_edge_width(edge_id);
	double T = get_time_step_length();
    /* keeping vehicles within the road */
	fyi = MAX(fyi, -WALL_DN(0, 1.05, vy, yi, wi));
	fyi = MIN(fyi, +WALL_UP(roadwid_meters, 1.05, vy, yi, wi));

    /* x wall */
    if (sim->dynamic_x_walls && sim->walls.x.leader != -1) {
        if (sim->walls.x.bound < fxi) {
            fxi = sim->walls.x.bound;
            sim->reg++;
        }
    }

    /* [umin, umax] ranges */
    fxi = MIN(fxi, sim->uxmax_hard);
    fxi = MAX(fxi, sim->uxmin_hard);

    fyi = MIN(fyi, +sim->uymax_hard);
    fyi = MAX(fyi, -sim->uymax_hard);

    /* non-negative speed */
    fxi = MAX(fxi, -vx/T);

    /* non-excessive speed */
    fxi = MIN(fxi, (sim->vd_alpha*vd - vx)/T);

    /* lat. speed never exceeds 0.5 x lon. speed, i.e.
     * enforcing: |vy[k+1]| <= 0.5 vx[k+1]
     */
    fyi = MIN(fyi, +(0.5*vx - vy)/T);
    fyi = MAX(fyi, -(0.5*vx + vy)/T);

    /* single-lane exceptions */
    if (sim->single_lane)
        fyi = 0;

	*fx = fxi;
	*fy = fyi;
}

void
determine_controls(sim_t *sim, double* fx, double* fy) {
    if (sim->zero_controls || (sim->lanedrop)) {//remove the && i==0 in the second term
		*fx = 0;
		*fy = 0;
        return;
    }
    *fx = MIN(*fx, sim->uxmax_hard);
    *fx = MAX(*fx, sim->uxmin_hard);
    *fy = (*fy >= 0)? MIN(*fy, sim->uymax_hard): MAX(*fy, -sim->uymax_hard);
}


void
sim_configure(sim_t *sim) {
	//read file	
	FILE *fp;
	fp = fopen("sim.conf" , "r");
	if(fp == NULL) {
      printf("Error opening file\n");
      return;
   	}
	char buf[1024];
	int input_int;
	double input_dbl;
	
	

	while (fgets(buf, sizeof(buf), fp)) {
		if (sscanf(buf, "alpha:%lf", &input_dbl) == 1)
			sim->alpha = input_dbl;
		if (sscanf(buf, "frame_timegap:%lf", &input_dbl) == 1)
			sim->frame_timegap = input_dbl;
		/* duet configuration */
		if (sscanf(buf, "duet:%lf", &input_dbl) == 1)
			sim->duet = input_dbl;
		if (sscanf(buf, "duet_vd:%lf", &input_dbl) == 1)
			sim->duet_vd = input_dbl;
		if (sscanf(buf, "duet_v0:%lf", &input_dbl) == 1)
			sim->duet_v0 = input_dbl;
		if (sscanf(buf, "duet_vd_slow:%lf", &input_dbl) == 1)
			sim->duet_vd_slow = input_dbl;
		if (sscanf(buf, "duet_dy:%lf", &input_dbl) == 1)
			sim->duet_dy = input_dbl;

		if (sscanf(buf, "four_lane_init:%d", &input_int) == 1)
			sim->four_lane_init = input_int;
		if (sscanf(buf, "ftsx_disabled:%d", &input_int) == 1)
			sim->ftsx_disabled = input_int;
		if (sscanf(buf, "fca_nbors_nudge:%d", &input_int) == 1)
			sim->fca_nbors_nudge = input_int;
		if (sscanf(buf, "fca_nbors:%d", &input_int) == 1)
			sim->fca_nbors = input_int;
		if (sscanf(buf, "fca_sat:%d", &input_int) == 1)
			sim->fca_sat = input_int;
		if (sscanf(buf, "fca_max:%d", &input_int) == 1)
			sim->fca_max = input_int;
		if (sscanf(buf, "barrier:%d", &input_int) == 1)
			sim->barrier = input_int;
		if (sscanf(buf, "single_lane:%d", &input_int) == 1)
			sim->single_lane = input_int;
		if (sscanf(buf, "single_lane_space:%lf", &input_dbl) == 1)
			sim->single_lane_space = input_dbl;
		if (sscanf(buf, "single_lane_front_vd:%lf", &input_dbl) == 1)
			sim->single_lane_front_vd = input_dbl;
		if (sscanf(buf, "dynamic_x_walls:%d", &input_int) == 1)
			sim->dynamic_x_walls = input_int;
			
		if (sscanf(buf, "dynamic_y_walls:%d", &input_int) == 1)
			sim->dynamic_y_walls = input_int;

		if (sscanf(buf, "fca_method:%d", &input_int) == 1)
			sim->fca_method = (fca_method_t)input_int;
		if (sscanf(buf, "lanedrop:%d", &input_int) == 1)
			sim->lanedrop = input_int;
		if (sscanf(buf, "zero_initial_speed:%d", &input_int) == 1)
			sim->zero_initial_speed = input_int;
		if (sscanf(buf, "zero_controls:%d", &input_int) == 1)
			sim->zero_controls = input_int;
		if (sscanf(buf, "virtual_lanes:%d", &input_int) == 1)
			sim->virtual_lanes = input_int;

		
		if (sscanf(buf, "n:%d", &input_int) == 1)
			sim->n = input_int;
		if (sscanf(buf, "K:%d", &input_int) == 1)
			sim->K = input_int;
		if (sscanf(buf, "T:%lf", &input_dbl) == 1)
			sim->T = input_dbl;
		
		if (sscanf(buf, "influence_radius_meters:%lf", &input_dbl) == 1)
			sim->influence_radius_meters= input_dbl;

		if (sscanf(buf, "time_gap_x:%lf", &input_dbl) == 1)
			sim->time_gap_x= input_dbl;
		if (sscanf(buf, "time_gap_y:%lf", &input_dbl) == 1)
			sim->time_gap_y= input_dbl;

		if (sscanf(buf, "coeff_fcax:%lf", &input_dbl) == 1)
			sim->coeff_fcax = input_dbl;
		if (sscanf(buf, "coeff_fcay:%lf", &input_dbl) == 1)
			sim->coeff_fcay = input_dbl;

		if (sscanf(buf, "fcax_zeta:%lf", &input_dbl) == 1)
			sim->fcax_zeta = input_dbl;
		if (sscanf(buf, "fcay_zeta:%lf", &input_dbl) == 1)
			sim->fcay_zeta = input_dbl;

		if (sscanf(buf, "safety_level_x:%lf", &input_dbl) == 1)
			sim->safety_level_x = input_dbl;

		if (sscanf(buf, "safety_level_y:%lf", &input_dbl) == 1)
			sim->safety_level_y = input_dbl;

		if (sscanf(buf, "fwd_force_max_x:%lf", &input_dbl) == 1)
			sim->fwd_force_max_x = input_dbl;
		if (sscanf(buf, "fwd_force_max_y:%lf", &input_dbl) == 1)
			sim->fwd_force_max_y = input_dbl;

		if (sscanf(buf, "bwd_force_max_x:%lf", &input_dbl) == 1)
			sim->bwd_force_max_x = input_dbl;
		if (sscanf(buf, "bwd_force_max_y:%lf", &input_dbl) == 1)
			sim->bwd_force_max_y = input_dbl;

		if (sscanf(buf, "coeff_fmdl:%lf", &input_dbl) == 1)
			sim->coeff_fmdl = input_dbl;

		if (sscanf(buf, "vd_meters_per_sec_lo:%lf", &input_dbl) == 1)
			sim->vd_meters_per_sec_lo = input_dbl;
		if (sscanf(buf, "vd_meters_per_sec_hi:%lf", &input_dbl) == 1)
			sim->vd_meters_per_sec_hi = input_dbl;
		if (sscanf(buf, "vd_meters_per_sec_lo_truck:%lf", &input_dbl) == 1)
			sim->vd_meters_per_sec_lo_truck = input_dbl;
		if (sscanf(buf, "vd_meters_per_sec_hi_truck:%lf", &input_dbl) == 1)
			sim->vd_meters_per_sec_hi_truck = input_dbl;

		if (sscanf(buf, "uxmax_hard:%lf", &input_dbl) == 1)
			sim->uxmax_hard = input_dbl;
		if (sscanf(buf, "uxmin_hard:%lf", &input_dbl) == 1)
			sim->uxmin_hard = input_dbl;
		if (sscanf(buf, "uymax_hard:%lf", &input_dbl) == 1)
			sim->uymax_hard = input_dbl;
		
		if (sscanf(buf, "roadlen_meters:%lf", &input_dbl) == 1)
			sim->roadlen_meters = input_dbl;
		if (sscanf(buf, "roadwid_meters:%lf", &input_dbl) == 1)
			sim->roadwid_meters = input_dbl;
		
		if (sscanf(buf, "ftsx_zeta:%lf", &input_dbl) == 1)
			sim->ftsx_zeta = input_dbl;
		if (sscanf(buf, "ftsx_hi:%lf", &input_dbl) == 1)
			sim->ftsx_hi = input_dbl;
		if (sscanf(buf, "ftsx_lo:%lf", &input_dbl) == 1)
			sim->ftsx_lo = input_dbl;

		if (sscanf(buf, "ftsy_zeta:%lf", &input_dbl) == 1)
			sim->ftsy_zeta = input_dbl;
		if (sscanf(buf, "ftsy_hi:%lf", &input_dbl) == 1)
			sim->ftsy_hi = input_dbl;
		if (sscanf(buf, "vd_alpha:%lf", &input_dbl) == 1)
			sim->vd_alpha = input_dbl;
	}
	
	if (!sim->alpha 
			||(sim->vd_throttling && !sim->vd_throttling_horizon)
			||!sim->coeff_fcax ||!sim->coeff_fcay ||!sim->fcax_zeta ||!sim->fcay_zeta ||!sim->safety_level_x || !sim->safety_level_y
			||!sim->vd_meters_per_sec_lo ||!sim->vd_meters_per_sec_hi
			||!sim->vd_meters_per_sec_lo_truck ||!sim->vd_meters_per_sec_hi_truck			
			||!sim->uxmax_hard ||!sim->uxmin_hard ||!sim->uymax_hard
			||!sim->ftsx_zeta || !sim->ftsx_hi || !sim->ftsx_lo 
			||!sim->ftsy_zeta || !sim->ftsy_hi || !sim->vd_alpha) {
		fprintf(stderr, "incomplete configuration\n");
		//close file
		fclose(fp);
		exit(1);
	}
	

	LANEWIDTH = 3.4;
	if (sim->four_lane_init)
		LANEWIDTH = 2.6;

	if (sim->duet)
		sim->n = 2;

	//sim->fca_nbors = MIN(sim->fca_nbors, sim->n);

	//close file
	fclose(fp);
}