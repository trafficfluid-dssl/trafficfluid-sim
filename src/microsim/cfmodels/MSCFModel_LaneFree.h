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

#include "MSCFModel.h"
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <utils/xml/SUMOXMLDefinitions.h>
//#include <tgmath.h> 
#define MAX_ITERS 5
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
typedef long long int NumericalID;
typedef struct ArrayMem arrayMemStruct;



double fRand(double fMin, double fMax);

//declare here the api functions ("C compatible", in terms if arguments, and return value) (maybe implement them directly, or inside the cpp file alternatively)

// NumericalID* lf_sim_get_all_ids();

// NumericalID lf_sim_get_all_ids_size();

// our LaneFree vehicle contains a pointer to the corresponding SUMO vehicle, while also containing information regarding the lateral speed and the desired (longitudinal) speed.
class MSLaneFreeVehicle{
public:
    static double last_init_pos_y;
    static double last_v_width;

    MSLaneFreeVehicle(MSVehicle* veh){
        myveh = veh;
        speed_y = 0;
        // veh->setSpeed(28);
        speed_x = veh->getSpeed();
        speed_x_desired = veh->getMaxSpeed();
        if(is_lanefree()){
            double road_width = veh->getEdge()->getWidth();
            double v_width = veh->getWidth();
            double random_init_y_pos;
            int max_it = 0;            
            if((v_width+last_v_width)<0.75*road_width){
                do{
                    random_init_y_pos = fRand(v_width/2, road_width-v_width/2);
                    max_it++;
                }while(abs(random_init_y_pos-last_init_pos_y)<(v_width/2+last_v_width/2) && max_it < MAX_ITERS);
            }
            else{
                random_init_y_pos = fRand(v_width/2, road_width-v_width/2);
            }
            last_init_pos_y = random_init_y_pos;
            last_v_width = v_width;
            double pos_y = get_position_y();
            double dist_from_lane = veh->getLateralPositionOnLane();
            double transformation = pos_y - dist_from_lane;
            double new_dist_from_lane = random_init_y_pos - transformation;
            veh->setLateralPositionOnLane(new_dist_from_lane);    
        }
        
        ring_road = false;
        accel_y = 0;
        accel_x = 0;
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

    bool get_ring_road(){
        return ring_road;
    }

    double get_desired_speed(){
        return speed_x_desired;
    }

    void set_desired_speed(double new_desired_speed){
        speed_x_desired = new_desired_speed;
    }

    
    double get_speed_x(){
        return myveh->getSpeed();
    }


    double get_speed_y(){
        return speed_y;
    }

    double get_position_x(){
        return myveh->getPositionOnLane()-myveh->getLength()/2;
    }

    double get_position_y(){
        double latOffset = 0;
        MSLane *mylane = myveh->getLane();
        MSEdge *myedge = &(mylane->getEdge());

        while((mylane = myedge->rightLane(mylane))){
            latOffset += mylane->getWidth();
        }
        return myveh->getLateralPositionOnLane()+((myveh->getLane()->getWidth())/2) + latOffset;        
    }

    void apply_acceleration(double ux, double uy){
        speed_x = myveh->getSpeed() + ux*TS;
        accel_y = uy;
        accel_x = ux;
        
    }

    // Complies with the CF model, returns the next speed of the vehicle. Lateral position is updated internally here
    double apply_acceleration_internal(){
                
        update_y(accel_y);
        update_x(accel_x);

        accel_y = 0;
        accel_x = 0;
        // std::cout<<"actual pos for veh " <<myveh->getID()<< " is:" <<myveh->getPosition() << "\n";
        return speed_x;
    }

    bool is_lanefree(){
        return ((myveh->getCarFollowModel().getModelID())==SUMO_TAG_CF_LANEFREE);
    }


protected:

    void update_y(double lateral_acceleration){
        double new_pos_y = myveh->getLateralPositionOnLane() + speed_y*TS + 0.5*lateral_acceleration*TS*TS;   
        myveh->setLateralPositionOnLane(new_pos_y);    
        speed_y = speed_y + lateral_acceleration*TS;
    }

    void update_x(double longitudinal_acceleration){
        if(!ring_road){
            return;
        }
        double new_pos_x = myveh->getPositionOnLane() + speed_x*TS + 0.5*longitudinal_acceleration*TS*TS;
        double edge_length = myveh->getEdge()->getLength();
        if(new_pos_x>=edge_length){
            new_pos_x = fmod(new_pos_x, edge_length);
            myveh->setPositionOnLane(new_pos_x);
        }
    }


    MSVehicle* myveh;
    double speed_x;
    double speed_y;
    double speed_x_desired;
    double accel_x;
    double accel_y;
    bool ring_road;
    

};



typedef std::unordered_map<NumericalID,MSLaneFreeVehicle*> VehicleMap;
typedef std::unordered_map<NumericalID,VehicleMap*> VehicleMapEdges;

typedef std::unordered_map<NumericalID,std::vector<double>> InsertedLateralInitStatus;

class LaneFreeSimulationPlugin{


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

    void add_new_lat_stats(NumericalID veh_id, double pos_y, double speed_y);

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

    arrayMemStruct* get_all_ids_in_edge_mem(){
        return &all_ids_in_edge;
    }
    arrayMemStruct* get_veh_type_name_mem(){
        return &veh_type_name;
    }
    arrayMemStruct* get_detector_ids_mem(){
        return &detector_ids;
    }
    arrayMemStruct* get_detector_name_mem(){
        return &detector_name;
    }
    arrayMemStruct* get_detector_values_mem(){
        return &detector_values;
    }
    arrayMemStruct* get_density_per_segment_per_edge_mem(){
        return &density_per_segment_per_edge;
    } 

    std::string get_message_step() {
        if (msgBufferVector.empty()) {
            return "";
        }
        std::string tot_msg = std::accumulate(msgBufferVector.begin(), msgBufferVector.end(), std::string(""));
        msgBufferVector.clear();
        return tot_msg;
    }

    void append_message_step(std::string msg) {
        msgBufferVector.push_back(msg);
    }

    
protected:
    NumericalID find_stored_edge(MSVehicle* veh);
    void free_hashmap();
    /// @brief Unique instance of LaneFreeSimulationPlugin
    static LaneFreeSimulationPlugin* myInstance;
    
    VehicleMapEdges allVehiclesMapEdges;

    InsertedLateralInitStatus insertedLatInitStatus;

    double max_vehicle_length;

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
    
    std::vector<std::string> msgBufferVector;
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

