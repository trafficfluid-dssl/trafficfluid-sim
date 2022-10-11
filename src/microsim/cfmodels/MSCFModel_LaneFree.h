/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2020 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSCFModel_LaneFree.h
/// @author
/// @date    Thu, 29 Oct 2020
///
// LaneFree model
/****************************************************************************/
#pragma once
#include <config.h>
#include <unordered_map>
#include <array>
#include "MSCFModel.h"
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <random>
#include <chrono>

//#ifdef __unix__
//#include "LaneFree_linux.h"
//#elif defined(WIN32)
//#include "LaneFree_win.h"
//#endif

//#include <tgmath.h> 
#define MAX_ITERS 5
//#define UPDATE_PRINT_MS 5
// TODO check consistency with https://sumo.dlr.de/docs/Developer/CodeStyle.html (use the suggested object instead of unordered_map & modify for loops with iterators)
// TODO check if we can just include LaneFree.h file, so as to not define NumericalID again here

//TODO convert enum to enum class
typedef enum {
    CHAR_M,
    INT_M,
    DOUBLE_M,
    NUMID_M
} ARRAYTYPE;

struct ArrayMem {
    void* ptr;
    size_t asize;
    size_t usize;
    bool updated;
    ARRAYTYPE type;
};

struct less_than_key
{
    inline bool operator() (const SUMOVehicle* v1, const SUMOVehicle* v2)
    {
        return (((MSVehicle*)v1)->getPositionOnLane() - ((MSVehicle*)v1)->getLength()/2) < (((MSVehicle*)v2)->getPositionOnLane() - ((MSVehicle*)v2)->getLength()/2);
    }
};


typedef long long int NumericalID;
typedef struct ArrayMem arrayMemStruct;


typedef std::unordered_map<NumericalID, std::array<double, 2>> LastVehicleStatus;

double fRand(double fMin, double fMax);


double lf_plugin_get_global_position_x(NumericalID veh_id);

//declare here the api functions ("C compatible", in terms if arguments, and return value) (maybe implement them directly, or inside the cpp file alternatively)

// NumericalID* lf_sim_get_all_ids();

// NumericalID lf_sim_get_all_ids_size();

// our LaneFree vehicle contains a pointer to the corresponding SUMO vehicle, while also containing information regarding the lateral speed and the desired (longitudinal) speed.
class MSLaneFreeVehicle{
public:
    //static double last_init_pos_y; //use the last vehicle status
    //static double last_v_width;
    static LastVehicleStatus last_veh_status;

    MSLaneFreeVehicle(MSVehicle* veh) : collided_with_cur{}, collided_with_prev{}{
        myveh = veh;
        speed_y = 0;
        // veh->setSpeed(28);
        speed_x = veh->getSpeed();
        speed_x_desired = veh->getDesiredSpeed();
        
        ring_road = false;
        accel_y = 0;
        accel_x = 0;
        myveh->initializeCachedGlobalPos();        
    }

    
    void just_collided_with(NumericalID c_veh_id) {
        collided_with_cur.insert(c_veh_id);
    }

    void empty_collided_set() {
        //collided_with_prev.clear();
        collided_with_prev = collided_with_cur;
        collided_with_cur.clear();
    }

    bool has_collided_with(NumericalID c_veh_id) {
        if (collided_with_prev.find(c_veh_id) != collided_with_prev.end()) {
            return true;
        }

        return false;
    }

    void set_position_x(double new_pos_x){
        myveh->setPositionOnLane(new_pos_x+myveh->getLength()/2);
    }

    void set_position_y(double new_pos_y){
        double pos_y = get_position_y();
        double dist_from_lane = myveh->getLateralPositionOnLane();
        double transformation = pos_y - dist_from_lane;
        double new_dist_from_lane = new_pos_y - transformation;
        myveh->setLateralPositionOnLane(new_dist_from_lane);   
    }

    void set_position_x_front(double new_pos_x) {
        myveh->setPositionOnLane(new_pos_x);
    }

    void set_position_y_front(double new_pos_y) {
        update_lane(new_pos_y);
    }

    void set_angle_relative(double new_angle_theta) {
        // std::cout << "initial angle:" << new_angle_theta << "\n";
        myveh->setAngleRelative(new_angle_theta);
    }
    void set_speed_x(double new_speed_x){
        myveh->setSpeed(new_speed_x);
    }
    void set_speed_y(double new_speed_y){
        speed_y = new_speed_y;
    }
    MSVehicle* get_vehicle(){
        return myveh;
    }

    void set_ring_road(bool circular){
        ring_road = circular;
    }

    bool is_circular(){
        return ring_road;
    }

    double get_desired_speed(){
        return speed_x_desired;
    }

    void set_desired_speed(double new_desired_speed){
        speed_x_desired = new_desired_speed;
        myveh->setDesiredSpeed(new_desired_speed);
    }

    
    double get_speed_x(){
        return myveh->getSpeed();
    }


    double get_speed_y(){
        return speed_y;
    }

    double get_position_x(){ 

        // consider non-zero orientation only for the bicycle model
        double cos_angle = (myveh->getVehicleType().getParameter().cmdModel != SUMO_TAG_LF_CMD_BICYCLE)  ? 1 : cos(myveh->getAngleRelative());

        double pos_x = myveh->getPositionOnLane() - (myveh->getLength() / 2)* cos_angle;

        //avoid negative positions when circular movement
        /*
        if (is_circular() && pos_x < 0) { // This creates issues when checking the order of vehicles
            return myveh->getLane()->getEdge().getLength() + pos_x;
        }

        */

        return pos_x;
    }

    double get_position_y(){ // generalize here for angle!
        double latOffset = 0;
        MSLane *mylane = myveh->getLane();
        MSEdge *myedge = &(mylane->getEdge());

        while((mylane = myedge->rightLane(mylane))){
            latOffset += mylane->getWidth();
        }

        // consider non-zero orientation only for the bicycle model
        double sin_angle = (myveh->getVehicleType().getParameter().cmdModel != SUMO_TAG_LF_CMD_BICYCLE) ? 0 : sin(myveh->getAngleRelative());

        return myveh->getLateralPositionOnLane() + ((myveh->getLane()->getWidth()) / 2) + latOffset - (myveh->getLength() / 2) * sin_angle;
    }

    void apply_acceleration(double ux, double uy){
        
        accel_y = uy;
        accel_x = ux;

        
        
    }

    void apply_acceleration_bicycle(double F, double delta) {

        // no need to define more parameters
        accel_x = F;
        accel_y = delta;

        
    }

    // Complies with the CF model, returns the next speed of the vehicle. Lateral position is updated internally here
    double apply_acceleration_internal(){
        if (myveh->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE) {
            apply_acceleration_internal_bicycle();
            myveh->setMyAccelerationBC(accel_x);
            myveh->setMyDeltaBC(accel_y);
        }
        else {
            update_y(accel_y);
            update_x(accel_x);
            myveh->setMyAccelerationLat(accel_y);
        }
        
        
        accel_y = 0;
        accel_x = 0;
        // std::cout<<"actual pos for veh " <<myveh->getID()<< " is:" <<myveh->getPosition() << "\n";
        return speed_x;
    }

    

    bool is_lanefree(){
        return ((myveh->getCarFollowModel().getModelID())==SUMO_TAG_CF_LANEFREE);
    }

    NumericalID get_edge_id() {
        return (myveh->getEdge()->getNumericalID());
    }

    // simply obtain the global coordinates for the back of the vehicle (as needed for the bicycle model)
    void get_global_coordinates_bicycle_model(double* x_pos, double* y_pos, double sigma, double theta_cur) {
        Position cachedGlobalPos = myveh->getCachedGlobalPos();
        *x_pos = cachedGlobalPos.x() - sigma * cos(theta_cur);
        *y_pos = cachedGlobalPos.y() - sigma * sin(theta_cur);
    }
protected:

    void update_lane(double new_pos_y) {
        //check whether we need to update the current lane
        //std::cout<<"veh "<< myveh->getID() << " in lane:"<<myveh->getLane()->getID()<<"\n";
        if ((abs(new_pos_y) > ((myveh->getLane()->getWidth()) / 2))) {
            MSLane* veh_lane = myveh->getLane();

            // use myveh->getLane()->getEdge() instead of myveh->getEdge(), since the latter does not incorporate internal edges (when vehicle is on junctions)
            const MSEdge* veh_edge = &myveh->getLane()->getEdge();
            MSLane* new_lane = nullptr;
            int direction = 0;
            if (new_pos_y > 0) {
                new_lane = veh_edge->leftLane(veh_lane);
                direction = 1;
                //std::cout << "go to left lane "<< new_lane->getID() <<" for " << myveh->getID() << "\n";
            }
            else if (new_pos_y < 0) {
                direction = -1;
                new_lane = veh_edge->rightLane(veh_lane);
                //std::cout << "go to right lane " << new_lane->getID() << " for " << myveh->getID() << "\n";
            }

            if (new_lane != nullptr && !(new_lane->isAccelLane())) {
                
                std::vector<std::pair<SUMOTime, int> > laneTimeLine;
                int laneIndex = myveh->getLaneIndex() + direction;
                laneTimeLine.push_back(std::make_pair(MSNet::getInstance()->getCurrentTimeStep(), laneIndex));
                laneTimeLine.push_back(std::make_pair(MSNet::getInstance()->getCurrentTimeStep(), laneIndex));
                myveh->getInfluencer().setLaneTimeLine(laneTimeLine);


                new_pos_y = new_pos_y - direction * (veh_lane->getWidth() + new_lane->getWidth()) / 2;                
            }

        }

        myveh->setLateralPositionOnLane(new_pos_y);
    }

    void update_y(double lateral_acceleration){
        double pos_on_lane = myveh->getLateralPositionOnLane();
        double new_pos_y = pos_on_lane + speed_y * TS + 0.5 * lateral_acceleration * TS * TS;
        
        
        //check whether we need to update the current lane
        update_lane(new_pos_y);
        
        speed_y = speed_y + lateral_acceleration * TS;
        
    }

    void update_x(double longitudinal_acceleration){
        

        //we use myveh->getSpeed() instead of the class variable speed_x, since speed_x already contains the updated value
        double new_pos_x = myveh->getPositionOnLane() + myveh->getSpeed() * TS + 0.5 * longitudinal_acceleration * TS * TS;
        
        double edge_length = myveh->getEdge()->getLength();
        double threshold = myveh->getLength() / 2;

        speed_x = myveh->getSpeed() + longitudinal_acceleration * TS;

        if (!ring_road) {
            return;
        }

        if(new_pos_x>=edge_length-threshold){
            new_pos_x = myveh->getPositionOnLane()-edge_length;
            myveh->setPositionOnLane(new_pos_x);
        }

        
    }

    

    
    void convert_to_local_coordinates(double* x_pos_local, double* y_pos_local, Position& pos, const MSLane* mylane) {


        //*x_pos_local = std::max(0., std::min(double(mylane->getLength() - POSITION_EPS),
        //    mylane->interpolateGeometryPosToLanePos(
        //        mylane->getShape().nearest_offset_to_point25D(pos, false))));
        *x_pos_local = mylane->interpolateGeometryPosToLanePos(mylane->getShape().nearest_offset_to_point25D(pos, false)); //mylane->interpolateGeometryPosToLanePos(mylane->getShape().nearest_offset_to_point25D(pos, false));

        //double angle_lane = mylane->getShape().beginEndAngle();
        //std::cout << *x_pos_local << " and angle:" << angle_lane << "\n";
        //std::cout << *x_pos_local << ", length:" << mylane->getLength() << "\n";
        //if (*x_pos_local - myveh->getPositionOnLane() < 3) {
        //    *x_pos_local = myveh->getPositionOnLane() + 3;
        //}
        const double perpDist = mylane->getShape().distance2D(pos, false);
        *y_pos_local = perpDist;// std::min(perpDist, 0.5 * (mylane->getWidth() + myveh->getVehicleType().getWidth() - MSGlobals::gLateralResolution));
        PositionVector tmp = mylane->getShape();
        tmp.move2side(-*y_pos_local); // moved to left
        if (tmp.distance2D(pos) > perpDist) {
            *y_pos_local = -*y_pos_local;
        }
    }


    void apply_acceleration_internal_bicycle() {
        // do all the updates here!, convert to positions on the back, update these first, and then convert them to the ones on the front

        double x_cur_back, y_cur_back, x_next_back, y_next_back, deltaPos_x_back, deltaPos_y_back, x_next_front, y_next_front, deltaPos_x_front, v_cur, v_next, theta_cur, theta_next, F, delta, sigma;
        F = accel_x;
        delta = accel_y;
        v_cur = myveh->getSpeed();
        
        theta_cur = myveh->getAngleRelative(); // is automatically local or global

        sigma = myveh->getLength();

        double tan_delta = tan(delta);

        double theta_next_elem_2 = v_cur * (tan_delta / sigma) * TS;
        double theta_next_elem_3 = F * (tan_delta / (2 * sigma)) * pow(TS, 2);


        v_next = v_cur + F * TS;

        bool global_coordinates=myveh->getGlobalCoordinatesControl();
        //printf("curr step local:\n x:%f,y:%f\n", get_position_x()+ (sigma / 2) * cos(theta_cur), get_position_y() + (sigma / 2) * sin(theta_cur));
        if (global_coordinates) {
            get_global_coordinates_bicycle_model(&x_cur_back, &y_cur_back, sigma, theta_cur);
            //Position pos_cur = myveh->getCachedGlobalPos();
            //printf("cached pos:(%f,%f) vs obtained pos:(%f,%f)", pos_cur.x() - (sigma)*cos(theta_cur), pos_cur.y() - (sigma)*sin(theta_cur), x_cur_back, y_cur_back);
            //printf("local (front):(%f,%f)\n",myveh->getPositionOnLane() ,myveh->getLateralPositionOnLane());
            //x_cur_back = pos_cur.x() - (sigma)*cos(theta_cur);
            //y_cur_back = pos_cur.y() - (sigma)*sin(theta_cur);
        }
        else {
            
            x_cur_back = get_position_x() - (sigma / 2) * cos(theta_cur); // convert to the positions on the back
            y_cur_back = get_position_y() - (sigma / 2) * sin(theta_cur); // convert to the positions on the back
        }
        //printf("within simulator for veh:%s\nx:%f,y:%f,v:%f,theta:%f\n", myveh->getID().c_str(), x_cur_back, y_cur_back, v_cur, theta_cur);
        // calculate delta of the angle, and then add it with respect to either global or local angle
        double delta_theta = theta_next_elem_2 + theta_next_elem_3;
        theta_next = theta_cur + delta_theta;

        
        if (delta == 0) {
            deltaPos_x_back = v_cur * cos(theta_cur) * TS + F * cos(theta_cur) * pow(TS, 2);


            deltaPos_y_back = v_cur * sin(theta_cur) * TS + F * sin(theta_cur) * pow(TS, 2);
        
        }
        else {
            deltaPos_x_back = (sigma) * (sin(theta_next)/tan_delta - sin(theta_cur)/tan_delta);
            

            deltaPos_y_back = (sigma) * (cos(theta_cur)/tan_delta - cos(theta_next)/tan_delta);
        
        }

        // vehicles are not allowed to move backwards        
        //deltaPos_x_back = (deltaPos_x_back < 0 ) ? 0 : deltaPos_x_back;

        x_next_back = x_cur_back + deltaPos_x_back;
        y_next_back = y_cur_back + deltaPos_y_back;

        x_next_front = x_next_back + (sigma)*cos(theta_next); // convert to the front bumper positions again to comply with sumo
        y_next_front = y_next_back + (sigma)*sin(theta_next); // convert to the front bumper positions again to comply with sumo
        //printf("next step:\n\t   x = %f, y = %f,v:%f,theta:%f\n", x_next_back, y_next_back, v_next, theta_next);
        if (global_coordinates) {
            myveh->setCachedGlobalPos(x_next_front, y_next_front);
            Position cachedGlobalPos = myveh->getCachedGlobalPos();
            const MSLane* mylane = myveh->getLane();
            //std::cout << "mylane:" << mylane->getID() << "\n";
            convert_to_local_coordinates(&x_next_front, &y_next_front, cachedGlobalPos, mylane);
            //std::cout << "next local:(" <<x_next_front<< "," << y_next_front<< ")\n";
        }
        //printf("next step local (front):\n x:%f,y:%f\n", x_next_front, y_next_front);
        
        // need to update speed_x parameter! It is more convenient to use the v speed value instead of the actual longitudinal speed of the vehicle
        speed_x = (v_next < 0) ? 0 : v_next;
        
        // used to update properly the new position of the vehicle (corresponds to the front bumper)
        deltaPos_x_front = x_next_front - myveh->getPositionOnLane();
        // vehicles are not allowed to move backwards     
        // The following line creates issues for global coordinates control   
        //deltaPos_x_front = (deltaPos_x_front < 0 ) ? 0 : deltaPos_x_front;
        //std::cout << "delta x: (local)" << deltaPos_x_front << "\n";
        
        // std::cout << "delta x:" << deltaPos_x_front<<"\n";
        // useful for compliance with existing structure, will update properly the longitudinal position
        myveh->setDeltaPosLF(deltaPos_x_front);
        
        //update lane properly, and set new latpos on lane etc
        double y_lane_next_front;
        if (global_coordinates) {
            y_lane_next_front = y_next_front;
        }
        else {
            double y_lane_cur_back = myveh->getLateralPositionOnLane() - (sigma)*sin(theta_cur);

            double y_lane_next_back = y_lane_cur_back + deltaPos_y_back;

            y_lane_next_front = y_lane_next_back + (sigma)*sin(theta_next);

            
        }
        
        update_lane(y_lane_next_front);
        

        //myveh->setLateralPositionOnLane(y_lane_next_front);

        
        // update theta angle
        myveh->setAngleRelative(theta_next);


        if (!ring_road) {
            return;
        }
        // if vehicle is in ring road
        double edge_length = myveh->getEdge()->getLength();
        double threshold = myveh->getLength() / 2;
        if (x_next_front >= edge_length - threshold) {
            x_next_front = myveh->getPositionOnLane() - edge_length;
            myveh->setPositionOnLane(x_next_front);
        }

    }


    MSVehicle* myveh;
    double speed_x;
    double speed_y;
    double speed_x_desired;
    double accel_x;
    double accel_y;
    bool ring_road;


    std::set<NumericalID> collided_with_prev;
    std::set<NumericalID> collided_with_cur;
};



typedef std::unordered_map<NumericalID, MSLaneFreeVehicle*> VehicleMap;
typedef std::unordered_map<NumericalID, VehicleMap*> VehicleMapEdges;

typedef std::vector<const SUMOVehicle*> SortedVehiclesVector;
typedef std::unordered_map<NumericalID, SortedVehiclesVector*> SortedVehiclesVectorEdges;

typedef std::unordered_map<NumericalID, std::vector<double>> InsertedAdditionalInitStatus;

class MSLaneFreeVehicle;

class LaneFreeSimulationPlugin {


    //pointer to all vehicles' vector

    //method of 
public:
    // assign function pointers to the declared functions

    // constructor
    LaneFreeSimulationPlugin();
    //destructor
    ~LaneFreeSimulationPlugin();

    void initialize_lib();
    void lf_simulation_step();

    // void sim_event_1(arguments);
    // void sim_event_2(arguments);
    // // etc
    void lf_simulation_checkCollisions();
    /** @brief Returns whether the network was already constructed
    * @return whether the network was already constructed
    */
    void finalize_event();

    static LaneFreeSimulationPlugin* getInstance();

    static bool hasInstance() {
        return myInstance != nullptr;
    }




    void insert_vehicle(MSVehicle* veh);
    void remove_vehicle(MSVehicle* veh);
    void change_edge(MSVehicle* veh);



    MSLaneFreeVehicle* find_vehicle(NumericalID veh_id);
    MSLaneFreeVehicle* find_vehicle_in_edge(NumericalID veh_id, NumericalID edge_id);
    NumericalID find_edge(NumericalID veh_id);

    void add_new_veh_additional_stats(NumericalID veh_id, double pos_x, double pos_y, double speed_y, double theta, bool use_global_coordinates);


    arrayMemStruct* get_all_ids_mem() {
        return &all_ids;
    }

    arrayMemStruct* get_lane_free_ids_mem() {
        return &lane_free_ids;
    }

    arrayMemStruct* get_vehicle_name_mem() {
        return &vehicle_name;
    }

    arrayMemStruct* get_all_edges_mem() {
        return &all_edges;
    }

    arrayMemStruct* get_edge_name_mem() {
        return &edge_name;
    }

    arrayMemStruct* get_all_ids_in_edge_mem() {
        return &all_ids_in_edge;
    }
    arrayMemStruct* get_veh_type_name_mem() {
        return &veh_type_name;
    }
    arrayMemStruct* get_detector_ids_mem() {
        return &detector_ids;
    }
    arrayMemStruct* get_detector_name_mem() {
        return &detector_name;
    }
    arrayMemStruct* get_detector_values_mem() {
        return &detector_values;
    }
    arrayMemStruct* get_density_per_segment_per_edge_mem() {
        return &density_per_segment_per_edge;
    }

    arrayMemStruct* get_all_neighbor_ids_front_mem() {
        return &all_neighbor_ids_front;
    }

    arrayMemStruct* get_all_neighbor_ids_back_mem() {
        return &all_neighbor_ids_back;
    }

    //Deprecated print
    //bool is_message_empty() {
    //    return msgBufferVector.empty();
    //}
    //Deprecated print
    //std::string get_message_step();

    //Deprecated print
    //void append_message_step(std::string msg);

    SortedVehiclesVector* get_sorted_vehicles_in_edge(NumericalID edge_id);

    double get_max_vehicle_length() {
        return max_vehicle_length;
    }

    double get_uniform_distribution_sample(double from, double to);



    // properly adapt existing codebase here, in order to convert global coordinates to local
    void convert_to_local_coordinates(double* x_pos_local, double* y_pos_local, Position& pos, const MSLane* mylane);

    // returns the execution time (in seconds) of the previous call for the simulation_step function 
    double get_last_step_exec_time() {
        return step_timer_seconds;
    }

    // returns the execution time (in seconds) of the previous step (disregarding the execution time for the simulation_step function, i.e., execution time for the SUMO application)
    double get_last_step_app_exec_time() {
        return rest_app_timer_seconds;
    }
    void get_all_neighbors_ring_road_internal(MSLaneFreeVehicle* lfveh, const MSEdge* current_edge, SortedVehiclesVector* current_edge_sorted_vehs, size_t veh_index, double distance, bool front, int cross_edge, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance);
    void get_all_neighbors_internal(MSLaneFreeVehicle* lfveh, const  MSEdge* current_edge, SortedVehiclesVector* current_edge_sorted_vehs, size_t veh_index, double distance, bool front, int cross_edge, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance);
protected:
    NumericalID find_stored_edge(MSVehicle* veh);
    void get_vehicles_from_other_direction_edges(NumericalID veh_id, double global_pox_x, double global_pos_y, double global_theta, bool front, const std::vector<MSLane*>& internal_lanes, NumericalID current_edge_id, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance);
    
    void transform_neighbor_vehicle_distance_and_add_to_neighbors(MSVehicle* veh_ptr, double global_pos_x, double global_pos_y, double cos_theta, double sin_theta, bool front, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance);
    void free_hashmap();
    /// @brief Unique instance of LaneFreeSimulationPlugin
    static LaneFreeSimulationPlugin* myInstance;

    VehicleMapEdges allVehiclesMapEdges;

    InsertedAdditionalInitStatus insertedAdditionalInitStatus;

    SortedVehiclesVectorEdges sortedVehiclesVectorEdges;

    double max_vehicle_length;
    double max_vehicle_width;
    double max_vehicle_diag;
    arrayMemStruct all_ids;
    arrayMemStruct lane_free_ids;
    arrayMemStruct vehicle_name;
    arrayMemStruct all_edges;
    arrayMemStruct edge_name;
    arrayMemStruct all_ids_in_edge;
    arrayMemStruct veh_type_name;
    arrayMemStruct detector_ids;
    arrayMemStruct detector_name;
    arrayMemStruct detector_values;
    arrayMemStruct density_per_segment_per_edge;

    arrayMemStruct all_neighbor_ids_front;
    arrayMemStruct all_neighbor_ids_back;

    std::default_random_engine random_engine;
    std::uniform_real_distribution<double> uniform_real_dis;
    //Deprecated print
    //std::vector<std::string> msgBufferVector;

    //long printMessageTimer;

    double step_timer_seconds;
    double rest_app_timer_seconds;

    std::chrono::steady_clock::time_point before_step_time;
    std::chrono::steady_clock::time_point after_step_time;
};




// we should include (#include "MSCFModel_LaneFree.h") in all sources that the object is used



// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_LaneFree
 * @brief The Intelligent Driver Model (IDM) car-following model
 * @see MSCFModel
 */
class MSCFModel_LaneFree : public MSCFModel {
public:
    /** @brief Constructor
     *  @param[in] vtype the type for which this model is built and also the parameter object to configure this model
     *  @param[in] idmm Wether IDM or IDMM shall be built
     */
    MSCFModel_LaneFree(const MSVehicleType* vtype, bool idmm);


    /** @brief Constructor
     * @param[in] accel The maximum acceleration
     * @param[in] decel The maximum deceleration
     * @param[in] emergencyDecel The maximum emergency deceleration
     * @param[in] apparentDecel The deceleration as expected by others
     * @param[in] headwayTime the headway gap
     * @param[in] adaptationFactor a model constant
     * @param[in] adaptationTime a model constant
     * @param[in] internalStepping internal time step size
     */
    MSCFModel_LaneFree(const MSVehicleType* vtype, double accel, double decel, double emergencyDecel, double apparentDecel,
                  double headwayTime, double adaptationFactor, double adaptationTime,
                  double internalStepping);


    /// @brief Destructor
    ~MSCFModel_LaneFree();


    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Applies interaction with stops and lane changing model influences
     * @param[in] veh The ego vehicle
     * @param[in] vPos The possible velocity
     * @return The velocity after applying interactions with stops and lane change model influences
     */
    double finalizeSpeed(MSVehicle* const veh, double vPos) const;

    /** @brief Computes the vehicle's safe speed without a leader
     *
     * Returns the velocity of the vehicle in dependence to the length of the free street and the target
     *  velocity at the end of the free range. If onInsertion is true, the vehicle may still brake
     *  before the next movement.
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] seen The look ahead distance
     * @param[in] maxSpeed The maximum allowed speed
     * @param[in] onInsertion whether speed at insertion is asked for
     * @return EGO's safe speed
     */
    virtual double freeSpeed(const MSVehicle* const veh, double speed, double seen,
                             double maxSpeed, const bool onInsertion = false) const;

    /** @brief Computes the vehicle's safe speed (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @return EGO's safe speed
     * @see MSCFModel::ffeV
     */
    double followSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const pred = 0) const;


    /** @brief Computes the vehicle's safe speed for approaching a non-moving obstacle (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] gap2pred The (netto) distance to the the obstacle
     * @return EGO's safe speed for approaching a non-moving obstacle
     * @see MSCFModel::ffeS
     * @todo generic Interface, models can call for the values they need
     */
    double stopSpeed(const MSVehicle* const veh, const double speed, double gap) const;


    /** @brief Returns the maximum gap at which an interaction between both vehicles occurs
     *
     * "interaction" means that the LEADER influences EGO's speed.
     * @param[in] veh The EGO vehicle
     * @param[in] vL LEADER's speed
     * @return The interaction gap
     * @todo evaluate signature
     * @see MSCFModel::interactionGap
     */
    double interactionGap(const MSVehicle* const, double vL) const;


    /** @brief Computes the vehicle's safe speed (no dawdling)
     * This method is used during the insertion stage. Whereas the method
     * followSpeed returns the desired speed which may be lower than the safe
     * speed, this method only considers safety constraints
     *
     * Returns the velocity of the vehicle in dependence to the vehicle's and its leader's values and the distance between them.
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @return EGO's safe speed
     */
    double insertionFollowSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const pred = 0) const;


    /** @brief Returns the minimum gap to reserve if the leader is braking at maximum (>=0)
     * @param[in] veh The vehicle itself, for obtaining other values
     * @param[in] pred The leader vehicle, for obtaining other values
     * @param[in] speed EGO's speed
     * @param[in] leaderSpeed LEADER's speed
     * @param[in] leaderMaxDecel LEADER's max. deceleration rate
     */
    double getSecureGap(const MSVehicle* const veh, const MSVehicle* const pred, const double speed, const double leaderSpeed, const double leaderMaxDecel) const;

    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
     */
    int getModelID() const {
        return SUMO_TAG_CF_LANEFREE;
    }
    /// @}



    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
     */
    MSCFModel* duplicate(const MSVehicleType* vtype) const;


    VehicleVariables* createVehicleVariables() const {
        if (myAdaptationFactor != 1.) {
            return new VehicleVariables();
        }
        return 0;
    }


private:
    class VehicleVariables : public MSCFModel::VehicleVariables {
    public:
        VehicleVariables() : levelOfService(1.) {}
        /// @brief state variable for remembering speed deviation history (lambda)
        double levelOfService;
    };


private:
    double _v(const MSVehicle* const veh, const double gap2pred, const double mySpeed,
              const double predSpeed, const double desSpeed, const bool respectMinGap = true) const;


private:
    /// @brief whether the model is IDMM or IDM
    const bool myIDMM;

    /// @brief The IDM delta exponent
    const double myDelta;

    /// @brief The IDMM adaptation factor beta
    const double myAdaptationFactor;

    /// @brief The IDMM adaptation time tau
    const double myAdaptationTime;

    /// @brief The number of iterations in speed calculations
    const int myIterations;

    /// @brief A computational shortcut
    const double myTwoSqrtAccelDecel;

private:
    /// @brief Invalidated assignment operator
    MSCFModel_LaneFree& operator=(const MSCFModel_LaneFree& s);
};

