#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <omp.h>

#define PRINT_XY_DEGREES 0
#define PARALLEL_RUN 1


#include "Controller.h"



#define BUS_RATIO 0
double LANEWIDTH;
#define VD_VARIANCE 2.0
#define XBUFFER 2.0

#define SAMPLE_UNIFORM(min, max) ((double)min + ((double)random()/RAND_MAX)*(max - min))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))




#define WALL_DN(point, safety, v) U_lemma3(sim, -(safety)*v, MAX(0, sim->y[i][t]-0.5*sim->w[i] - (point)), -sim->uymax_hard)
#define WALL_UP(point, safety, v) U_lemma3(sim, +(safety)*v, MAX(0, (point) - (sim->y[i][t]+0.5*sim->w[i])), -sim->uymax_hard)



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
fmdl(sim_t *sim, NumericalID veh_id) {
    double vd_range = (sim->vd_meters_per_sec_hi - sim->vd_meters_per_sec_lo);
    double point_in_range = 0.5;
    double spread_factor = 1.0;
    double gravity_point;
    if (sim->virtual_lanes) {
        point_in_range = (get_desired_speed(veh_id) - sim->vd_meters_per_sec_lo)/vd_range;
    }

	double width = get_veh_width(veh_id), yv = get_position_y(veh_id);
    gravity_point = width/2 + (0 + point_in_range)*(sim->roadwid_meters - width)*spread_factor;
    return sim->coeff_fmdl*(2 / (1 + exp(-sim->ftsy_zeta*(gravity_point - yv))) - 1);
}

static double
U_lemma3(sim_t *sim, double v, double d, double ubar) {
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
	doubly wi = get_veh_width(veh_i), wj = get_veh_width(veh_j);
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
        approaching_speed_y = fabs(vyj - vyi);
    }

    mag = fca_mag(sim,
                get_relative_position_x(veh_j, veh_i), y-y0, 
                vxi,
                approaching_speed_x, approaching_speed_y, len, wid);
    
    ang = atan2(dy_i_to_j, dx_i_to_j);
    *fcax = -cos(ang) * mag;
    *fcay = -sin(ang) * mag;

    return mag;
}


static void
walls_init(sim_t *sim, NumericalID veh_id, NumericalID edge_id, walls_t *walls) {
	double roadlen_meters = get_edge_length(edge_id), roadwid_meters = get_edge_width(edge_id);
	double T = get_current_time_step();
	double vx = get_speed_x(veh_id), vy = get_speed_y(veh_id);
	walls->x.position = fmod(get_position_x(veh_id) + 0.5* roadlen_meters, roadlen_meters);
    walls->x.distance = 0.5* roadlen_meters;
    walls->x.velocity = vx;
    walls->x.leader = -1;
    walls->x.bound = (get_desired_speed(veh_id)- vx) /T;
    walls->x.degree = 0;

    walls->y.dn = 0;
    walls->y.up = roadwid_meters;

    walls->y.up_j = -1;
    walls->y.dn_j = -1;

	
    walls->y.bound_dn = WALL_DN(0, sim->safety_level_y, vy);
    walls->y.bound_up = WALL_UP(roadwid_meters, sim->safety_level_y, vy);

    walls->y.degree_up = 0;
    walls->y.degree_dn = 0;
}

static void
walls_update(sim_t *sim, NumericalID veh_i, NumericalID veh_j, double fcax, double fcay, walls_t *walls) {
    if (j == i) 
        return;
    
    // clip forces within [-1, 1]
    fcax = MIN(1.0, MAX(-1.0, fcax));
 
    // degree_x in [0, 1]: degree to which j is a front obstacle
    double degree_x = pow(fabs(MIN(fcax, 0)), 0.25);
	
	double vxi = get_speed_x(veh_i), vxj = get_speed_x(veh_j);
	double vdi = get_desired_speed(veh_i);

	double T = get_current_time_step();

	double lj = get_veh_length(veh_j), xj = get_position_x(veh_j);

    // determine front bound
	double dx = get_relative_position_x(veh_i, veh_j) - (get_veh_length(veh_i) + get_veh_length(veh_j))/2;//circular_dx(sim, sim->x[j][t]-0.5*sim->l[j] - (sim->x[i][t]+0.5*sim->l[i]));
    double vs = MIN(dx/sim->frame_timegap, vxj - 0.1);
    double uxbound = ((1.-degree_x)*(vdi-vxi) + degree_x*(vs - vxi))/(1*T);

    if (degree_x > walls->x.degree) {
        walls->x.bound = uxbound;
        walls->x.leader = j;
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

typedef struct {
    int j;
    double mag;
    double fcax, fcay;
} nbor_t;

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
determine_forces(sim_t *sim, NumericalID edge_id, int i, NumericalID* vehs_array, int n, double* fx, double* fy) {
    int j;
    double fcax = 0, fcay = 0;
    nbor_t nbors[n];
    nbor_t nbors_nudge[n];

    memset(nbors, 0, sizeof(nbors));
    memset(nbors_nudge, 0, sizeof(nbors_nudge));

    
    walls_init(sim, edge_id, i, vehs_array, &(sim->walls[i]));
    /* initalize forces to zero */
	
	double fxi = 0, fyi = 0;

    /* obstacle-related forces */
    double fcax_sum = 0, fcay_sum =0;
    int nbors_added = 0, nbors_added_nudge = 0;
	NumericalID veh_id = vehs_array[i];
    for (j=0; j < n; j++) {
		if (j == i)
			continue;
        double dx_i_to_j = get_relative_position_x(veh_id, vehs_array[j]);
        double dy_i_to_j = get_relative_position_y(veh_id, vehs_array[j]);
        double fcamag = 0;

        
        //TODO fix this, this is O(n^2), we can exploit the ordered vehicles structure
        if (fabs(dx_i_to_j > sim->influence_radius_meters))
            continue;
        
        fcamag = fca(sim, vehs_array[i], vehs_array[j], dx_i_to_j, dy_i_to_j, &fcax, &fcay);
        
#if(PRINT_XY_DEGREES == 2)
        sim->degree_x[i][t][j] = fcax;
        sim->degree_y[i][t][j] = fcay;
#endif

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

            walls_update(sim, vehs_array[i], vehs_array[nbors[x].j], t, nbors[x].fcax, nbors[x].fcay, &(sim.walls));
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

    /* target-speed related forces */
    if (!sim->ftsx_disabled)
        fxi += ftsx(sim, i, t);
    fyi += ftsy(sim, i, t);
    fyi += fmdl(sim, i, t);

    sim->wall_y_up = sim->walls.y.up;
    sim->wall_y_dn = sim->walls.y.dn;
    sim->wall_y_up_j = sim->walls.y.up_j;
    sim->wall_y_dn_j = sim->walls.y.dn_j;


	*fx = fxi;
	*fy = fyi;
}

static void
regulate_forces(sim_t *sim, NumericalID edge_id, NumericalID veh_id, double* fx, double* fy) {
    /* y wall */
	double fxi = *fx, fyi = *fy;
    if (sim->dynamic_y_walls && i != 0) {
        double uy_max = sim->walls.y.bound_up;
        double uy_min = sim->walls.y.bound_dn;
        if (uy_max >= -uy_min) {
			fyi = MAX(fyi, -uy_min);
			fyi = MIN(fyi, +uy_max);
        }
    }

	double vx = get_speed_x(veh_id), vd = get_desired_speed(veh_id), vy = get_speed_y(veh_id);
	double roadwid_meters = get_edge_width(edge_id);
	double T = get_current_time_step();
    /* keeping vehicles within the road */
	fyi = MAX(fyi, -WALL_DN(0, 1.05, vy));
	fyi = MIN(fyi, +WALL_UP(roadwid_meters, 1.05, vy));

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

static void
determine_controls(sim_t *sim, double* fx, double* fy) {
    if (sim->zero_controls || (sim->lanedrop && i == 0)) {
		*fx = 0;
		*fy = 0;
        return;
    }
    *fx = MIN(*fx, sim->uxmax_hard);
    *fx = MAX(*fx, sim->uxmin_hard);
    *fy = (*fy >= 0)? MIN(*fy, sim->uymax_hard): MAX(*fy, -sim->uymax_hard);
}



static void
update_control(sim_t *sim, int t, int n, double *u_x, double *u_y){

	for (int i = 0; i < n; i++){
		u_x[i] = sim->ux[i][t];
		u_y[i] = sim->uy[i][t];
		
	}
	
}



static void vehicle_lw(sim_t *sim, int i, int choice){
	
	sim->l[i] = sim->class_l[choice];	
	sim->w[i] = sim->class_w[choice];
	
	
}


/* initalize dimensions */
static void
sim_initialize_lw(sim_t *sim)
{
	int i;

	for (i=0; i < sim->n; i++) {
		if (sim->lanedrop && i == 0) {
			sim->l[i] = 0.25 * sim->roadlen_meters;
			sim->w[i] = 3.0;
		} else if (sim->barrier && i == sim->n-1) {
			sim->l[i] = 2.5;
			sim->w[i] = sim->roadwid_meters;
		} else {
			if (SAMPLE_UNIFORM(0, 100) < BUS_RATIO)
				sim->truck[i] = 1;
			
			vehicle_lw(sim, i, 0);
		}
	}
}

/* initialize positions */
static void
sim_initialize_xy(sim_t *sim)
{
	const double lanewidth = LANEWIDTH;
	//const int numlanes = MIN(3, round(sim->roadwid_meters / lanewidth));
	const int numlanes = round(sim->roadwid_meters / lanewidth);

	int i, l, j;
	int *lastcar = calloc(sizeof(int), numlanes);
	int *numcars = calloc(sizeof(int), numlanes);

	for (l=0; l < numlanes; l++)
		lastcar[l] = -1;
 	
	/* well-structured placement */
	
	for (i=0, l=0; i < sim->n; i++, l++) {
		if (l == numlanes)
			l = 0;
		j = lastcar[l];
		sim->x[i][0] = XBUFFER + 0.5*sim->l[i] + ((j == -1)?0:sim->x[j][0]+0.5*sim->l[j]); 
		sim->y[i][0] = l*lanewidth + 0.5 + 0.5*sim->w[i];
 		lastcar[l] = i;
		numcars[l]++;
		/*
		for (j=0; j < i; j++) {
			if (i == j)
				continue;
			if (fabs(circular_dx(sim, sim->x[i][0]-sim->x[j][0])) < 0.5*(sim->l[i]+sim->l[j]) 
				&& fabs(sim->y[i][0] - sim->y[j][0]) < 0.5*(sim->w[i]+sim->w[j])) {
				fprintf(stderr, "Overlap while placing %d\n", i);
				exit(1);
			}
		}*/
	}

	/* determine random distribution of the slack on each lane */
	double **slack = calloc(sizeof(double*), numlanes);
	int v;
	for (l=0; l < numlanes; l++) {
		if ((j = lastcar[l]) == -1)
			continue;
		slack[l] = calloc(sizeof(double), numcars[l]);
		double total = 0;
		for (v=0; v < numcars[l]; v++)
			total += (slack[l][v] = (double)rand()/RAND_MAX);

		for (v=0; v < numcars[l]; v++)
			slack[l][v] /= total;

		total = sim->roadlen_meters - (sim->x[lastcar[l]][0]+0.5*sim->l[lastcar[l]]);
		double s = 0;
		for (v=0; v < numcars[l]; v++) {
			s += slack[l][v];
			slack[l][v] = s * total;
		}
	}

	/* distribute slack over cars on each lane */
	double **slackptr = calloc(sizeof(double*), numlanes);
	for (l=0; l < numlanes; l++)
			slackptr[l] = &slack[l][0];

	for (j=0; j < sim->n; j++) {
		l = j % numlanes;
		sim->x[j][0] += *(slackptr[l]++);
		sim->y[j][0] += 0.5*(double)rand()/RAND_MAX;

	}

	/* convert trucks that are positioned in the 3/4 of the road to cars */
	for (i = 0; i < sim->n; i++){
		if (sim->truck[i] == 1 && sim->y[i][0] < (1.0/3.0)*sim->roadwid_meters){
			vehicle_lw(sim, i, 0);
			sim->truck[i] = 0;
		}
	}
	

	for (l=0; l < numlanes; l++)
		free(slack[l]);

	free(slackptr);
	free(slack);
	free(lastcar);
	free(numcars);
}

static void
sim_initialize_xy_duet(sim_t *sim)
{
	sim->x[0][0] = 0.5*sim->l[0];
	sim->x[1][0] = sim->l[0] + 0.5*sim->l[1] + sim->duet;

	sim->y[0][0] = sim->roadwid_meters/2 + sim->duet_dy;
	sim->y[1][0] = sim->roadwid_meters/2;
}

/* initalize desired speeds */
static void
sim_initialize_vd(sim_t *sim)
{
	int i;

	for (i=0; i < sim->n; i++) {
		if (sim->lanedrop && i == 0) {
			sim->vx[i][0] = 0;
			sim->vd[i] = 0;
		} else {

			double lo = sim->vd_meters_per_sec_lo, hi = sim->vd_meters_per_sec_hi;
			double lo_truck = sim->vd_meters_per_sec_lo_truck, hi_truck = sim->vd_meters_per_sec_hi_truck;
			sim->vd[i] = lo + (1. - (sim->y[i][0]/sim->roadwid_meters))*(hi-lo);
			if (sim->truck[i] == 1){
				sim->vd[i] = lo_truck+ (1. - ((sim->y[i][0] - (1.0/3.0)*sim->roadwid_meters)/(sim->roadwid_meters*2.0/3.0)))*(hi_truck-lo_truck);
				//fprintf(stderr, "(%d) truck_vd: %f -- truck_y: %f -- track_x: %f \n", i, sim->vd[i], sim->y[i][0], sim->x[i][0]);
			}
			if (sim->zero_initial_speed)
				sim->vx[i][0] = 0;
			else
				sim->vx[i][0] = 0.5*(sim->vd_meters_per_sec_lo + sim->vd_meters_per_sec_hi);
		}
	}
}

static void
sim_initialize_vd_duet(sim_t *sim)
{
	sim->vx[0][0] = sim->duet_v0;	
	sim->vd[0] = sim->duet_vd;
	sim->vd[1] = sim->vx[1][0] = sim->duet_vd_slow;
}

static void
sim_initialize_xy_single(sim_t *sim)
{
	int i;
	double dx = 0;
	for (i=0; i < sim->n; i++) {
		sim->x[i][0] = 0.5*sim->l[i] + dx;
		dx = sim->x[i][0] + 0.5*sim->l[i] + sim->single_lane_space;

		if (dx > sim->roadlen_meters) {
			fprintf(stderr, "single-lane mode: unable to fit vehicle nr. %d\n", i);
			exit(1);
		}

		sim->y[i][0] = 0.5*sim->roadwid_meters;
	}
}

static void
sim_initialize_vd_single(sim_t *sim)
{
	sim_initialize_vd(sim);
	sim->vdx_effective[sim->n-1] = sim->vd[sim->n-1] = sim->single_lane_front_vd;
}


//we need to remove this!
void
sim_initialize(sim_t *sim) {
	

	assert((sim->walls = calloc(sizeof(walls_t*), sim->n)));


	assert((sim->wall_y_up = calloc(sizeof(double*), sim->n)));
	assert((sim->wall_y_dn = calloc(sizeof(double*), sim->n)));
	assert((sim->wall_y_up_j = calloc(sizeof(double*), sim->n)));
	assert((sim->wall_y_dn_j = calloc(sizeof(double*), sim->n)));

	assert((sim->crash = calloc(sizeof(int*), sim->n)));
	assert((sim->flow = calloc(sizeof(double), sim->K)));
	assert((sim->reg = calloc(sizeof(int), sim->K)));

	assert((sim->l = calloc(sizeof(double), sim->n)));
	assert((sim->w = calloc(sizeof(double), sim->n)));
	assert((sim->vd = calloc(sizeof(double), sim->n)));
	assert((sim->truck = calloc(sizeof(int), sim->n)));
	assert((sim->vdx_effective = calloc(sizeof(double), sim->n)));
	assert((sim->vdy_effective = calloc(sizeof(double), sim->n)));

	assert((sim->degree_x = calloc(sizeof(double**), sim->n)));
	assert((sim->degree_y = calloc(sizeof(double**), sim->n)));

	for (i=0; i < sim->n; i++) {
		assert((sim->x[i] = calloc(sizeof(double), sim->K)));
		assert((sim->y[i] = calloc(sizeof(double), sim->K)));

		assert((sim->vx[i] = calloc(sizeof(double), sim->K)));
		assert((sim->vy[i] = calloc(sizeof(double), sim->K)));

		assert((sim->ux[i] = calloc(sizeof(double), sim->K)));
		assert((sim->uy[i] = calloc(sizeof(double), sim->K)));

		assert((sim->fx[i] = calloc(sizeof(double), sim->K)));
		assert((sim->fy[i] = calloc(sizeof(double), sim->K)));

		assert((sim->walls[i] = calloc(sizeof(walls_t), sim->K)));

		assert((sim->wall_y_up[i] = calloc(sizeof(double), sim->K)));
		assert((sim->wall_y_dn[i] = calloc(sizeof(double), sim->K)));
		assert((sim->wall_y_up_j[i] = calloc(sizeof(double), sim->K)));
		assert((sim->wall_y_dn_j[i] = calloc(sizeof(double), sim->K)));

		assert((sim->crash[i] = calloc(sizeof(int), sim->K)));

#if(PRINT_XY_DEGREES)
		assert((sim->degree_x[i] = calloc(sizeof(double*), sim->K)));
		assert((sim->degree_y[i] = calloc(sizeof(double*), sim->K)));
		int t;
		for (t=0; t < sim->K; t++) {
			assert((sim->degree_x[i][t] = calloc(sizeof(double), sim->n)));
			assert((sim->degree_y[i][t] = calloc(sizeof(double), sim->n)));
		}
#endif

	}

	sim_initialize_lw(sim);
	if (sim->duet > 0) {
		sim_initialize_xy_duet(sim);
		sim_initialize_vd_duet(sim);
	} else if (sim->single_lane) {
		sim_initialize_xy_single(sim);
		sim_initialize_vd_single(sim);
	} else {
		sim_initialize_xy(sim);
		sim_initialize_vd(sim);
	}

#if 0

	// crash resolution period
	do {
		int j, crash_found, crash_resolution_round = 0;
		double ratio_backup = sim->push_repel_ratio;

		sim->warmup = 1;
		sim->push_repel_ratio = 1.1;

 		do {
			crash_found = 0;
			for (i=0; i < sim->n; i++) {
				for (j=0; j < i; j++) {
					if (crash(sim, i, j, 0)) {
						crash_found = 1;
					}
				}
			}
 			sim_run(sim, 2);

			for (i=0; i < sim->n; i++) {
				sim->x[i][0] = sim->x[i][1];
				sim->y[i][0] = sim->y[i][1];
				sim->vx[i][0] = sim->vy[i][0] = sim->ux[i][0] = sim->uy[i][0] = 0;
			}

			crash_resolution_round++;
			fprintf(stderr, "crash resolution rounds: %d\r", crash_resolution_round);
 		} while(crash_found);
		fprintf(stderr, "\n");

		for (i=0; i < sim->n; i++)
			memset(sim->crash[i], 0, sizeof(int)*sim->K);

		sim->warmup = 0;
		sim->push_repel_ratio = ratio_backup;

	}while(0);
#endif
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
			sim->fca_method = input_int;
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

void
sim_run(sim_t *sim, int t, int n, size_t* veh_id, int* class_id, double* v_d, double* pos_x, double* pos_y, double* v_x, double* v_y, double* u_x, double* u_y) {
    int i;
    double sensor_location = sim->roadlen_meters / 2;
    assert(n<=sim->n);

	//example for veh_id usage
	/*
	for(i=0;i<n;i++){
		printf("Vehicle id: %d\n",(int)veh_id[i]);
	}
	*/
	
    //for (t=0; t < MIN(K, sim->K)-1; t++) {
    sim->flow[t+1] = sim->flow[t];

    for (i=0; i < n; i++){
        vehicle_lw(sim, i, class_id[i]);        
        sim->vd[i] = v_d[i];
        sim->x[i][t] = pos_x[i];
        sim->y[i][t] = pos_y[i];
        sim->vx[i][t] = v_x[i];
        sim->vy[i][t] = v_y[i];
    }
	
    #if(PARALLEL_RUN)
    #pragma omp parallel for
    #endif
    for (i=0; i < n; i++){    
        determine_forces(sim, i, t, n);        
    }
    
    #if(PARALLEL_RUN)
    #pragma omp parallel for
    #endif
    for (i=0; i < n; i++) {
        regulate_forces(sim, i, t);
		
        determine_controls(sim, i, t);

        /*
        double T = sim->alpha * sim->T;
        sim->vx[i][t+1] = sim->vx[i][t] + sim->ux[i][t]*T; 
        sim->x[i][t+1] = sim->x[i][t] + sim->vx[i][t]*T + sim->ux[i][t]*0.5*pow(T, 2);
        sim->x[i][t+1] = fmod(sim->x[i][t+1], sim->roadlen_meters);
        */

        // update flow
        if (sim->x[i][t] <= sensor_location && sim->x[i][t+1] > sensor_location) {
            #pragma omp critical
            sim->flow[t+1]++;
        }
        /*
        if (!sim->single_lane) {
            sim->vy[i][t+1] = sim->vy[i][t] + sim->uy[i][t]*T; 
            sim->y[i][t+1] = sim->y[i][t] + sim->vy[i][t]*T + sim->uy[i][t]*0.5*pow(T, 2);
        } else {
            sim->y[i][t+1] = sim->y[i][t];
            sim->vy[i][t+1] = 0;
        }*/
    }
    //}
	
    update_control(sim, t, n, u_x, u_y);
	

    //if (!sim->warmup)
    //    fprintf(stderr,"time-step %5d crashes %5d\r", t, sim->crashes);
    
    //for (t=0; t < MIN(K, sim->K)-1; t++) 
    //  sim->flow[t] *= 3600/(t * sim->T);
    
}

void test_sim(sim_t *sim){
	printf("This is a test that shows sim is initialized, n:%d\tx:%f\n",sim->n,sim->x[2][0]);
		
}

void get_flow(sim_t *sim, double* flow, int size){
	for(int t=0;t<size;t++){
		flow[t] = sim->flow[t];
	}
}

