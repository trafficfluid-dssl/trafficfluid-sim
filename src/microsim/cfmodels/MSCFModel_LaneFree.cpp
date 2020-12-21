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
/// @file    MSCFModel_LaneFree.cpp
/// @author  
/// @date    Thu, 29 Oct 2020
///
// LaneFree Plugin model.
/****************************************************************************/
//#include <config.h>
//#include <bits/stdc++.h>

#include "MSCFModel_LaneFree.h"
#include <microsim/MSVehicle.h>
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include "LaneFree_win.h"
#endif

#include <microsim/MSNet.h>
#include <microsim/MSVehicleControl.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/lcmodels/MSLCM_LC2013.h>
#include <utils/common/SysUtils.h>
#include <microsim/output/MSMeanData_Net.h>

#include <libsumo/Vehicle.h>

LaneFreeSimulationPlugin* LaneFreeSimulationPlugin::myInstance = nullptr;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}



void initialise_arraymemory(arrayMemStruct* s, ARRAYTYPE atype) {
	s->ptr = NULL;
	s->asize = 0;
	s->usize = 0;
	s->updated = false;
	s->type = atype;

}

void update_arraymemory_size(arrayMemStruct* s, size_t requested_size) {
	if (requested_size <= s->asize) {
		return;
	}
	size_t block_size = 0;
	switch (s->type) {
	case NUMID_M:
		block_size = sizeof(NumericalID);
		break;
	case INT_M:
		block_size = sizeof(int);
		break;
	case CHAR_M:
		block_size = sizeof(char);
		break;
	case DOUBLE_M:
		block_size = sizeof(double);
		break;
	default:
		std::cout << "Type of array memory structure not recognized!\n";
		return;
	}


	if (s->ptr == NULL) {
		s->ptr = malloc(requested_size * block_size);
		s->asize = requested_size;
	}
	else if (requested_size > s->asize) {
		//std::cout << "Try to update size!\n";
		size_t allocated_blocks = std::max(requested_size, 2 * s->asize);
		s->ptr = realloc(s->ptr, allocated_blocks * block_size);
		
		s->asize = allocated_blocks; 
	}

	if (s->ptr == NULL) {
		std::cout << "Memory could not be allocated!\n";
	}

}


//implementation of every provided function from the API

// get all vehicles' ids inside the network
NumericalID* lf_plugin_get_all_ids(){
	// std::vector<std::string> ids;
	

    MSVehicleControl& c = MSNet::getInstance()->getVehicleControl();
    NumericalID ids_size = c.loadedVehSize();
	arrayMemStruct* all_ids_ams = LaneFreeSimulationPlugin::getInstance()->get_all_ids_mem();
	update_arraymemory_size(all_ids_ams, (size_t)ids_size);
    NumericalID* all_ids_array = (NumericalID*) all_ids_ams->ptr;
    int id_index = 0;
    for (MSVehicleControl::constVehIt i = c.loadedVehBegin(); i != c.loadedVehEnd(); ++i) {
        if ((*i).second->isOnRoad()) {
			all_ids_array[id_index] = ((*i).second)->getNumericalID();
        	id_index++;                
        }
        
    }
	all_ids_ams->updated = true;
	all_ids_ams->usize = (size_t)id_index;
    return all_ids_array;



}

// get the number of all vehicles inside the network
NumericalID lf_plugin_get_all_ids_size(){
	arrayMemStruct* all_ids_ams = LaneFreeSimulationPlugin::getInstance()->get_all_ids_mem();
	if (all_ids_ams->updated) {
		return all_ids_ams->usize;
	}

	MSVehicleControl& c = MSNet::getInstance()->getVehicleControl();
        
    int counter = 0;
    for (MSVehicleControl::constVehIt i = c.loadedVehBegin(); i != c.loadedVehEnd(); ++i) {
        if ((*i).second->isOnRoad()) {
            counter++;
            
        }
        
    }

    return counter;
}


// get all lanefree vehicles' ids inside the network
NumericalID* lf_plugin_get_lane_free_ids(){
	// std::vector<std::string> ids;
	

	MSVehicleControl& c = MSNet::getInstance()->getVehicleControl();
	NumericalID ids_size = c.loadedVehSize();
	arrayMemStruct* lane_free_ids_ams = LaneFreeSimulationPlugin::getInstance()->get_lane_free_ids_mem();
	update_arraymemory_size(lane_free_ids_ams, (size_t)ids_size);

	NumericalID* lf_ids_array = (NumericalID*)lane_free_ids_ams->ptr;
	int id_index = 0;
	for (MSVehicleControl::constVehIt i = c.loadedVehBegin(); i != c.loadedVehEnd(); ++i) {
		if ((*i).second->isOnRoad() && ((*i).second->getVehicleType().getCarFollowModel().getModelID()) == SUMO_TAG_CF_LANEFREE) {
			lf_ids_array[id_index] = ((*i).second)->getNumericalID();
			id_index++;
		}

	}
	lane_free_ids_ams->updated = true;
	lane_free_ids_ams->usize = (size_t)id_index;

	return lf_ids_array;

}

// get the number of all lanefree vehicles inside the network
NumericalID lf_plugin_get_lane_free_ids_size(){
	arrayMemStruct* lane_free_ids_ams = LaneFreeSimulationPlugin::getInstance()->get_lane_free_ids_mem();
	if (lane_free_ids_ams->updated) {
		return lane_free_ids_ams->usize;
	}

	MSVehicleControl& c = MSNet::getInstance()->getVehicleControl();
	NumericalID id_cnt = 0;
	for (MSVehicleControl::constVehIt i = c.loadedVehBegin(); i != c.loadedVehEnd(); ++i) {
        if ((*i).second->isOnRoad() && ((*i).second->getVehicleType().getCarFollowModel().getModelID())==SUMO_TAG_CF_LANEFREE){        	
        	id_cnt++;
        }
    }
    return id_cnt;

}

char* lf_plugin_get_vehicle_name(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		return NULL;
	}
	
	std::string vname =  lfveh->get_vehicle()->getID();
	int l = vname.length()+1;

	arrayMemStruct* vehicle_name_ams = LaneFreeSimulationPlugin::getInstance()->get_vehicle_name_mem();
	update_arraymemory_size(vehicle_name_ams, (size_t)l);

	char* vn = (char*) vehicle_name_ams->ptr;
	if (vn != NULL) {
		strcpy(vn, vname.c_str());
	}
	
	return vn;
}
// get the ids of all roads inside the network
NumericalID* lf_plugin_get_all_edges(){
	
	arrayMemStruct* all_edges_ams = LaneFreeSimulationPlugin::getInstance()->get_all_edges_mem();
	//will not change dynamically over time
	if (all_edges_ams->updated) {
		return (NumericalID*)all_edges_ams->ptr;
	}
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	NumericalID edges_size = edges_v.size();
		
	update_arraymemory_size(all_edges_ams, (size_t)edges_size);

	NumericalID* edge_ids_array = (NumericalID*)all_edges_ams->ptr;
	int i = 0;
	if (edge_ids_array != NULL) {
		for (MSEdge* edge : edges_v) {
			edge_ids_array[i] = edge->getNumericalID();
			i++;
		}
	}
	all_edges_ams->updated = true;
	all_edges_ams->usize = (size_t)edges_size;
	return edge_ids_array;
    
}

// get the number of all roads inside the network
NumericalID lf_plugin_get_all_edges_size(){
	arrayMemStruct* all_edges_ams = LaneFreeSimulationPlugin::getInstance()->get_all_edges_mem();
	if (all_edges_ams->updated) {
		return all_edges_ams->usize;
	}
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	return edges_v.size();
	
}

char* lf_plugin_get_edge_name(NumericalID edge_id){
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	
	bool found = false;
	std::string ename;
	for (MSEdge* edge : edges_v) {
		if(edge->getNumericalID()==edge_id){
			found = true;
			ename = edge->getID();
			break;
		}
		
	}

	if(!found){
		std::cout<<"Edge with id "<< edge_id << "not found!\n";
		return NULL;
	}

	
	
	int l = ename.length()+1;
	arrayMemStruct* edge_name_ams = LaneFreeSimulationPlugin::getInstance()->get_edge_name_mem();
	update_arraymemory_size(edge_name_ams, (size_t)(l));
	char* en = (char*) edge_name_ams->ptr;
	if (en != NULL) {
		strcpy(en, ename.c_str());
	}

	return en;
}


// get the vehicles' ids that reside in a given road
NumericalID* lf_plugin_get_all_ids_in_edge(NumericalID edge_id){
	
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	if( (NumericalID)edges_v.size()<=edge_id){
		std::cout<< "Edge_id " << edge_id << "too large!\n";
		return nullptr;
	}	
	MSEdge* thisedge;
	bool found_edge = false;
	for (MSEdge* edge : edges_v) {
		if(edge->getNumericalID()==edge_id){
			thisedge = edge;
			found_edge = true;
			break;
		}
	}

	if(!found_edge){
		std::cout<< "Edge_id " << edge_id << "not found!\n";
		return nullptr;
	}

	std::vector<const SUMOVehicle*> vehs = thisedge->getVehicles();
	//TODO: this sort is performed twice (once here, once for collision check)
	std::sort(vehs.begin(), vehs.end(), less_than_key());
	size_t veh_size = vehs.size();
	arrayMemStruct* all_ids_in_edge_ams = LaneFreeSimulationPlugin::getInstance()->get_all_ids_in_edge_mem();
	update_arraymemory_size(all_ids_in_edge_ams, veh_size);

	int i = 0;
	NumericalID* all_ids_in_edge_array = (NumericalID*)	all_ids_in_edge_ams->ptr;
	if(all_ids_in_edge_array != NULL){
		for (const SUMOVehicle* veh : vehs){
			
			all_ids_in_edge_array[i] = veh->getNumericalID();
			i++;
			
		}
	}

	all_ids_in_edge_ams->updated = true;
	all_ids_in_edge_ams->usize = veh_size;
	return all_ids_in_edge_array;




}

NumericalID lf_plugin_get_edge_of_vehicle(NumericalID veh_id){
	NumericalID e_id = LaneFreeSimulationPlugin::getInstance()->find_edge(veh_id);
	return e_id;
}

// get the number of vehicles that reside in a given road
NumericalID lf_plugin_get_all_ids_in_edge_size(NumericalID edge_id){
	//arrayMemStruct* all_ids_in_edge_ams = LaneFreeSimulationPlugin::getInstance()->get_all_ids_in_edge_mem();
	//if (all_ids_in_edge_ams->updated) {
	//	return all_ids_in_edge_ams->usize;
	//}
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	if( (NumericalID)edges_v.size()<=edge_id){
		std::cout<< "Edge_id " << edge_id << "too large!\n";
		return -1;
	}	
	MSEdge* thisedge;
	bool found_edge = false;
	for (MSEdge* edge : edges_v) {
		if(edge->getNumericalID()==edge_id){
			thisedge = edge;
			found_edge = true;
			break;
		}		
	}

	if(!found_edge){
		std::cout<< "Edge_id " << edge_id << "not found!\n";
		return -1;
	}
	std::vector<const SUMOVehicle*> vehs = thisedge->getVehicles();
	return (NumericalID)vehs.size();
	

}

// get the longitudinal position of a given vehicle with respect to the road 
double lf_plugin_get_position_x(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout << "Vehicle not found!\n";
		return -1;
	}

	return lfveh->get_position_x();
}

double lf_plugin_get_relative_distance_x(NumericalID ego_id, NumericalID other_id) {
	MSLaneFreeVehicle* ego_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(ego_id);
	if (ego_lfveh == nullptr) {
		std::cout << "Ego not found!\n";
		return -1;
	}
	MSLaneFreeVehicle* other_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(other_id);
	if (other_lfveh== nullptr) {
		std::cout << "Other not found!\n";
		return -1;
	}

	if (ego_lfveh->get_edge_id() != other_lfveh->get_edge_id()) {
		std::cout << "Vehicles are not on the same edge! Relative distance betweeen vehicles on different road edges is not currently supported!\n";
		return -1;
	}
	double dx = other_lfveh->get_position_x() - ego_lfveh->get_position_x();

	if (ego_lfveh->is_circular()) {
		double roadlength = ego_lfveh->get_vehicle()->getEdge()->getLength();
		
		if (fabs(dx) <= 0.5 * roadlength)
			return dx;
		else
			return (dx >= 0) ? (dx - roadlength) : (dx + roadlength);
	}

	return dx;
}

double lf_plugin_get_relative_position_x(NumericalID ego_id, NumericalID other_id) {
	MSLaneFreeVehicle* ego_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(ego_id);
	if (ego_lfveh == nullptr) {
		std::cout << "Ego not found!\n";
		return -1;
	}
	MSLaneFreeVehicle* other_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(other_id);
	if (other_lfveh == nullptr) {
		std::cout << "Other not found!\n";
		return -1;
	}

	if (ego_lfveh->get_edge_id() != other_lfveh->get_edge_id()) {
		std::cout << "Vehicles are not on the same edge! Relative distance betweeen vehicles on different road edges is not currently supported!\n";
		return -1;
	}
	
	double dx = other_lfveh->get_position_x() - ego_lfveh->get_position_x();

	if (ego_lfveh->is_circular()) {
		double roadlength = ego_lfveh->get_vehicle()->getEdge()->getLength();

		if (fabs(dx) > 0.5 * roadlength){

			dx = (dx >= 0) ? (dx - roadlength) : (dx + roadlength);
		}
	}
	double r_x = ego_lfveh->get_position_x() + dx;
	return r_x;
}

double lf_plugin_get_relative_distance_y(NumericalID ego_id, NumericalID other_id) {
	MSLaneFreeVehicle* ego_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(ego_id);
	if (ego_lfveh == nullptr) {
		std::cout << "Ego not found!\n";
		return -1;
	}
	MSLaneFreeVehicle* other_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(other_id);
	if (other_lfveh == nullptr) {
		std::cout << "Other not found!\n";
		return -1;
	}

	if (ego_lfveh->get_edge_id() != other_lfveh->get_edge_id()) {
		std::cout << "Vehicles are not on the same edge! Relative distance betweeen vehicles on different road edges is not currently supported!\n";
		return -1;
	}
	double dy = other_lfveh->get_position_y() - ego_lfveh->get_position_y();

	return dy;
}



void lf_plugin_set_circular_movement(NumericalID veh_id, bool circular) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return;
	}

	lfveh->set_ring_road(circular);
}


// get the lateral position of a given vehicle (corresponds to the distance of the vehicle's center from the right road boundary)
double lf_plugin_get_position_y(NumericalID veh_id){

	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		return -1;
	}

	return lfveh->get_position_y();
	
}

// get the longitudinal speed of a given vehicle
double lf_plugin_get_speed_x(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		return -1;
	}

	return lfveh->get_speed_x();
	
}

// get the lateral speed of a given vehicle (positive when moving towards the left boundary)
double lf_plugin_get_speed_y(NumericalID veh_id){

	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		return -1;
	}

	return lfveh->get_speed_y();
	
}

// set the desired speed of a given vehicle
void lf_plugin_set_desired_speed(NumericalID veh_id, double new_d_speed){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		return;
	}

	lfveh->set_desired_speed(new_d_speed);

	
}
// get the desired speed of a given vehicle
double lf_plugin_get_desired_speed(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		return -1;
	}

	return lfveh->get_desired_speed();
	
}


// apply longitudinal & lateral acceleration on a given vehicle
void lf_plugin_apply_acceleration(NumericalID veh_id, double accel_x, double accel_y){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	lfveh->apply_acceleration(accel_x, accel_y);

}




// get the length of a given road
double lf_plugin_get_edge_length(NumericalID edge_id){
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	
	bool found = false;
	MSEdge* fedge=nullptr;
	for (MSEdge* edge : edges_v) {
		if(edge->getNumericalID()==edge_id){
			found = true;
			fedge = edge;
			break;
		}
		
	}

	if(!found){
		std::cout<<"Edge with id "<< edge_id << "not found!\n";
		return -1;
	}

	
	
    
	return fedge->getLength();

}


// get the width of a given road
double lf_plugin_get_edge_width(NumericalID edge_id){
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	
	bool found = false;
	MSEdge* fedge=nullptr;
	for (MSEdge* edge : edges_v) {
		if(edge->getNumericalID()==edge_id){
			found = true;
			fedge = edge;
			break;
		}
		
	}

	if(!found){
		std::cout<<"Edge with id "<< edge_id << "not found!\n";
		return -1;
	}

	
	
    
	return fedge->getWidth();

}

/*
//set the color of zero speed
void lf_plugin_set_zero_speed_color(double r, double g, double b){

}


//set the color of desired speed
void lf_plugin_set_desired_speed_color(double r, double g, double b){
	
}

// set the color of max_speed
void lf_plugin_set_max_speed_color(double r, double g, double b, double max_speed){
	
}
*/
// get the width of the veh_id vehicle
double lf_plugin_get_veh_length(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout<< "Vehicle with id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_vehicle()->getVehicleType().getLength();
}

// get the width of the veh_id vehicle
double lf_plugin_get_veh_width(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout<< "Vehicle with id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_vehicle()->getVehicleType().getWidth();

}

// get the time-step length
double lf_plugin_get_time_step_length(){
	return TS;
}

// get the current time-step
int lf_plugin_get_current_time_step(){
	
	return SIMSTEP/DELTA_T;//round(SIMTIME/TS);
}



// get the vehicle type id
NumericalID lf_plugin_get_veh_type_id(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout<< "Vehicle with id:" << veh_id << " not found!\n";
		return -1;
	}
	return lfveh->get_vehicle()->getVehicleType().getNumericalID();

}

// get the vehicle type string
char* lf_plugin_get_veh_type_name(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout<< "Vehicle with id:" << veh_id << " not found!\n";
		return NULL;
	}
	std::string vtname =  lfveh->get_vehicle()->getVehicleType().getID();


	int l = vtname.length()+1;
	arrayMemStruct* veh_type_name_ams = LaneFreeSimulationPlugin::getInstance()->get_veh_type_name_mem();
	update_arraymemory_size(veh_type_name_ams, (size_t)l);

	char* vtn = (char*)veh_type_name_ams->ptr;
	if (vtn != NULL) {
		strcpy(vtn, vtname.c_str());
	}
	
    
	return vtn;
}


// get the random seed parameter of SUMO
int lf_plugin_get_seed(){
	OptionsCont& oc = OptionsCont::getOptions();
	return oc.getInt("seed");
}

NumericalID lf_plugin_insert_new_vehicle(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y){
	
	// 
	std::string id(veh_name);
	std::string routeID(route_id);
	std::string vTypeID(type_id);
	std::string depart("now");
	std::string departLane("best");

	MSVehicleType* vehicleType = MSNet::getInstance()->getVehicleControl().getVType(vTypeID);
	double depart_pos_front = pos_x + vehicleType->getLength()/2;
	std::string departPos = std::to_string(depart_pos_front);
	std::string departSpeed = std::to_string(speed_x);
	NumericalID new_vid = libsumo::Vehicle::addR(id, routeID, vTypeID, depart, departLane, departPos, departSpeed);
	LaneFreeSimulationPlugin::getInstance()->add_new_lat_stats(new_vid, pos_y, speed_y);
	// MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(new_vid);
	// if(lfveh==nullptr){
	// 	std::cout<< "Vehicle with id:" << new_vid << " not found!\n";
	// 	return NULL;
	// }

	// lfveh->set_position_x(pos_x);
	// lfveh->set_position_y(pos_y);
	// lfveh->set_speed_x(speed_x);


	return new_vid;

}


NumericalID* lf_plugin_get_detectors_ids(){
	
	arrayMemStruct* detectors_ids_ams = LaneFreeSimulationPlugin::getInstance()->get_detector_ids_mem();
	//will not change dynamically over time
	if (detectors_ids_ams->updated) {
		return (NumericalID*)detectors_ids_ams->ptr;
	}
	MSDetectorControl* detectorControl = &MSNet::getInstance()->getDetectorControl();
	NumericalID count = 0;
	std::vector<NumericalID> d_ids;
	for(SumoXMLTag i: detectorControl->getAvailableTypes()){
		// std::cout<<i<<"\n";
		for (auto j : detectorControl->getTypedDetectors(i)) {            
            d_ids.push_back(count);
            count++;            
        }
	}
	if(d_ids.empty()){
		return NULL;	
	}
	
	int l = d_ids.size();
	
	update_arraymemory_size(detectors_ids_ams, (size_t)l);


	NumericalID* c_ids = (NumericalID*)detectors_ids_ams->ptr;
	
	std::copy(d_ids.begin(), d_ids.end(), c_ids);

	detectors_ids_ams->updated = true;
	detectors_ids_ams->usize = (size_t)l;

	return c_ids;


}


NumericalID lf_plugin_get_detectors_size(){
	arrayMemStruct* detectors_ids_ams = LaneFreeSimulationPlugin::getInstance()->get_detector_ids_mem();
	if (detectors_ids_ams->updated) {
		return detectors_ids_ams->usize;
	}

	MSDetectorControl* detectorControl = &MSNet::getInstance()->getDetectorControl();
	NumericalID count = 0;
	std::vector<NumericalID> d_ids;
	for(SumoXMLTag i: detectorControl->getAvailableTypes()){
		
		for (auto j : detectorControl->getTypedDetectors(i)) {           
           
            count++;            
        }
	}
	
	return count;


}
/*
void lf_plugin_print_to_sumo(char* msg) {
	std::string string_msg(msg);
	LaneFreeSimulationPlugin::getInstance()->append_message_step(string_msg);
	
	
}*/

char* lf_plugin_get_detector_name(NumericalID d_id){

	MSDetectorControl* detectorControl = &MSNet::getInstance()->getDetectorControl();
	NumericalID count = 0;
	std::string d_name;
	bool found = false;
	for(SumoXMLTag i: detectorControl->getAvailableTypes()){
		// std::cout<<i<<"\n";
		for (auto j : detectorControl->getTypedDetectors(i)) {            
            if(count==d_id){
            	d_name = ((MSInductLoop*)j.second)->getID();
            	found = true;
            	break;
            } 
			count++;
        }
        if(found){
        	break;
        }
	}

	if(!found){
		std::cout<<"Detector with id "<< d_id << " not found!\n";
		return NULL;	
	}


	int l = d_name.length()+1;

	arrayMemStruct* detector_name_ams = LaneFreeSimulationPlugin::getInstance()->get_detector_name_mem();
	update_arraymemory_size(detector_name_ams, (size_t)l);

	char* vtn = (char*) detector_name_ams->ptr;
	
	if (vtn != NULL) {
		strcpy(vtn, d_name.c_str());
	}	


	return vtn;
}

int* lf_plugin_get_detectors_values(){

	MSDetectorControl* detectorControl = &MSNet::getInstance()->getDetectorControl();
	int count;
	std::vector<int> values;
	for(SumoXMLTag i: detectorControl->getAvailableTypes()){
		// std::cout<<i<<"\n";
		for (auto j : detectorControl->getTypedDetectors(i)) {
            count = ((MSInductLoop*)j.second)->getVehiclesCount();
            values.push_back(count);
            // std::cout<<"detectors:" << count<<"\n";
        }
	}
	if(values.empty()){
		return NULL;	
	}
	size_t l = values.size();
	arrayMemStruct* detector_values_ams = LaneFreeSimulationPlugin::getInstance()->get_detector_values_mem();
	update_arraymemory_size(detector_values_ams,l);

	int* c_values = (int*)detector_values_ams->ptr;	
	
	std::copy(values.begin(), values.end(), c_values);

	detector_values_ams->updated = true;
	detector_values_ams->usize = l;

	return c_values;
	
}

int* lf_plugin_get_density_per_segment_per_edge(NumericalID edge_id, double segment_length){

	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();
	
	int i, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	MSVehicle* veh;
	

	int* dens_per_segment;
	int size_segments, segment_i;
	double x_v;
	for (MSEdge* edge : edges_v) {
		if(edge_id!=edge->getNumericalID()){
			continue;
		}
		vehs_in_edge = edge->getVehicles();
		n_v = vehs_in_edge.size();
		if(n_v==0){
			//std::cout<<"Edge with id "<< edge_id << " is empty!\n";
			return NULL;
		}
		size_segments =  (int)std::ceil(edge->getLength()/segment_length);	
		arrayMemStruct* dens_per_segment_ams = LaneFreeSimulationPlugin::getInstance()->get_density_per_segment_per_edge_mem();
		update_arraymemory_size(dens_per_segment_ams, size_segments);

		dens_per_segment = (int*)dens_per_segment_ams->ptr;
		
		
		for (i = 0; i < n_v; i++) {
			veh = (MSVehicle*)vehs_in_edge[i];
			x_v = veh->getPositionOnLane();
			segment_i = std::min((int)std::floor(x_v / segment_length), size_segments - 1);
			dens_per_segment[segment_i] += 1;

		
		}
		
		return dens_per_segment;
	}
	std::cout<<"Edge with id "<< edge_id << " not found!\n";
	return NULL;
}

int lf_plugin_get_density_per_segment_per_edge_size(NumericalID edge_id, double segment_length){

	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();
	
	// int i,j, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	// MSVehicle* veh;
	

	// int* dens_per_segment;
	int size_segments;
	// double x_v;
	for (MSEdge* edge : edges_v) {
		if(edge_id!=edge->getNumericalID()){
			continue;
		}
		vehs_in_edge = edge->getVehicles();
		if(vehs_in_edge.size()==0){
			//std::cout<<"Edge with id "<< edge_id << " is empty!\n";
			return -1;
		}
		size_segments =  (int)std::ceil(edge->getLength()/segment_length);		
		return size_segments;
	}
	std::cout<<"Edge with id "<< edge_id << " not found!\n";
	return -1;
}


LaneFreeSimulationPlugin::LaneFreeSimulationPlugin(){

	// assign the function pointers of the API to the corresponding implemented functions

	get_all_ids = &lf_plugin_get_all_ids; 
	get_lane_free_ids = &lf_plugin_get_lane_free_ids;
	get_all_ids_size = &lf_plugin_get_all_ids_size; 
	get_lane_free_ids_size = &lf_plugin_get_lane_free_ids_size;
	get_vehicle_name = &lf_plugin_get_vehicle_name;
	get_all_edges = &lf_plugin_get_all_edges;
	get_all_edges_size = &lf_plugin_get_all_edges_size;
	get_all_ids_in_edge = &lf_plugin_get_all_ids_in_edge;
	get_all_ids_in_edge_size = &lf_plugin_get_all_ids_in_edge_size;
	apply_acceleration = &lf_plugin_apply_acceleration;
	get_position_x = &lf_plugin_get_position_x;
	get_position_y = &lf_plugin_get_position_y;
	get_relative_distance_x = &lf_plugin_get_relative_distance_x;
	get_relative_position_x = &lf_plugin_get_relative_position_x;
	get_relative_distance_y = &lf_plugin_get_relative_distance_y;
	set_circular_movement = &lf_plugin_set_circular_movement;
	get_speed_y = &lf_plugin_get_speed_y;
	get_speed_x = &lf_plugin_get_speed_x;
	get_speed_y = &lf_plugin_get_speed_y;
	get_desired_speed = &lf_plugin_get_desired_speed;
	set_desired_speed = &lf_plugin_set_desired_speed;
	get_veh_type_id = &lf_plugin_get_veh_type_id; 
	get_veh_type_name = &lf_plugin_get_veh_type_name; 
	get_time_step_length = &lf_plugin_get_time_step_length;
	get_current_time_step = &lf_plugin_get_current_time_step;
	get_seed = &lf_plugin_get_seed;
	get_detectors_ids = &lf_plugin_get_detectors_ids;
	get_detectors_size = &lf_plugin_get_detectors_size;
	get_detector_name = &lf_plugin_get_detector_name;
	get_detectors_values = &lf_plugin_get_detectors_values;
	get_density_per_segment_per_edge = &lf_plugin_get_density_per_segment_per_edge;
	get_density_per_segment_per_edge_size = &lf_plugin_get_density_per_segment_per_edge_size;
	insert_new_vehicle = &lf_plugin_insert_new_vehicle;
	//Legacy code, just use printf
	//print_to_sumo = &lf_plugin_print_to_sumo;
	//printMessageTimer = SysUtils::getCurrentMillis();
	get_veh_length = &lf_plugin_get_veh_length;
	get_veh_width = &lf_plugin_get_veh_width;
	get_edge_length = &lf_plugin_get_edge_length;
	get_edge_width = &lf_plugin_get_edge_width;
	srand(lf_plugin_get_seed());
	max_vehicle_length = 0;
	
	// Initialize all pointers, and set corresponding counters to zero, counters reflect the allocated memory blocks
	
	initialise_arraymemory(&all_ids,NUMID_M);	
	initialise_arraymemory(&lane_free_ids, NUMID_M);
	initialise_arraymemory(&vehicle_name,CHAR_M);
	initialise_arraymemory(&all_edges, NUMID_M);
	initialise_arraymemory(&edge_name, CHAR_M);
	initialise_arraymemory(&all_ids_in_edge, NUMID_M);
	initialise_arraymemory(&veh_type_name, CHAR_M);
	initialise_arraymemory(&detector_ids, NUMID_M);
	initialise_arraymemory(&detector_name, CHAR_M);
	initialise_arraymemory(&detector_values,DOUBLE_M);
	initialise_arraymemory(&density_per_segment_per_edge,INT_M);
	myInstance = this;
}

void free_mem(void* ptr) {
	if (ptr != NULL) {
		free(ptr);
	}
}

LaneFreeSimulationPlugin::~LaneFreeSimulationPlugin() {
	simulation_finalize();
	free_hashmap();
	
	free_mem(all_ids.ptr);
	free_mem(lane_free_ids.ptr);
	free_mem(vehicle_name.ptr);
	free_mem(all_edges.ptr);
	free_mem(all_ids_in_edge.ptr);
	free_mem(veh_type_name.ptr);
	free_mem(detector_ids.ptr);
	free_mem(detector_name.ptr);
	free_mem(detector_values.ptr);
	free_mem(density_per_segment_per_edge.ptr);
	myInstance = nullptr;
}

void
LaneFreeSimulationPlugin::initialize_lib(){
	simulation_initialize();
}

/*
* Deprecated print
void 
LaneFreeSimulationPlugin::append_message_step(std::string msg){
	msgBufferVector.push_back(msg);
}

std::string
LaneFreeSimulationPlugin::get_message_step(){
	if (msgBufferVector.empty()) {
		return "";
	}
	std::string tot_msg;
	for (const auto& str_i : msgBufferVector) {
		tot_msg += str_i;
	}
	
	msgBufferVector.clear();
	return tot_msg;
}
*/
void
LaneFreeSimulationPlugin::lf_simulation_step(){
	
	all_ids.updated = false;
	
	lane_free_ids.updated = false;
	
	//edges do not change dynamically
	//all_edges.updated = false;
	
	all_ids_in_edge.updated = false;
	
	//similarly for detector ids
	//detector_ids.updated = false;
	
	detector_values.updated = false;

	density_per_segment_per_edge.updated = false;
	
	
	simulation_step();
	lf_simulation_checkCollisions();
	/*
	Deprecated print
	bool is_empty = is_message_empty();
	long timer = SysUtils::getCurrentMillis() - printMessageTimer;
	bool update_time;
	
	update_time = true ? timer > UPDATE_PRINT_MS : false;
	
	if ((!is_empty) && update_time) {
		std::string msg = get_message_step();
		
		
		
		if (MSNet::getInstance()->isGUINet()) {
			
			WRITE_MESSAGE(msg);
		}
		else{
			std::cout << msg << "\n";
		}


		if (update_time) {
			printMessageTimer = SysUtils::getCurrentMillis();
		}
	}
	*/
	

}


void
LaneFreeSimulationPlugin::add_new_lat_stats(NumericalID veh_id, double pos_y, double speed_y){
	std::vector<double> lat_array = {pos_y,speed_y};
	lat_array.push_back(pos_y);
	lat_array.push_back(speed_y);
	insertedLatInitStatus.insert(std::make_pair(veh_id,lat_array));
}



void
LaneFreeSimulationPlugin::lf_simulation_checkCollisions(){

	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();
	
	int i,j, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	MSVehicle* veh1;
	MSVehicle* veh2;
	MSLaneFreeVehicle* lfv1;
	MSLaneFreeVehicle* lfv2;
	double xv1, yv1, lv1, wv1, dx, dy;
	
	for (MSEdge* edge : edges_v) {
		vehs_in_edge = edge->getVehicles();
		std::sort(vehs_in_edge.begin(), vehs_in_edge.end(), less_than_key());
		n_v = vehs_in_edge.size();
		for(i=0;i<n_v-1;i++){
			veh1 = (MSVehicle*)vehs_in_edge[i];			
			lv1 = veh1->getVehicleType().getLength();
			wv1 = veh1->getVehicleType().getWidth();
			lfv1 = find_vehicle_in_edge(veh1->getNumericalID(),edge->getNumericalID());
			xv1 = lfv1->get_position_x();
			yv1 = lfv1->get_position_y();
			for(j=i+1;j<n_v;j++){
				veh2 = (MSVehicle*)vehs_in_edge[j];
				lfv2 = find_vehicle_in_edge(veh2->getNumericalID(),edge->getNumericalID());				
				dx = abs(xv1-lfv2->get_position_x());
				dy = abs(yv1-lfv2->get_position_y());
				if((dx<((lv1+veh2->getVehicleType().getLength())/2)) && (dy<((wv1/2+veh2->getVehicleType().getWidth())/2))){
					event_vehicles_collide(veh1->getNumericalID(), veh2->getNumericalID());
				}
				if(dx>(lv1+max_vehicle_length)/2){
					break;
				}
			}
			// std::cout<<"veh "<< veh->getID() << " on pos " << veh->getPositionOnLane() << "\n";
		}
	}
	
	
}


void
LaneFreeSimulationPlugin::free_hashmap(){
	VehicleMap *vm_edge;
	MSLaneFreeVehicle * lf_veh;
	for(VehicleMapEdges::iterator it=allVehiclesMapEdges.begin();it!=allVehiclesMapEdges.end();it++){
		vm_edge = it->second;
		for(VehicleMap::iterator it_v=vm_edge->begin();it_v!=vm_edge->end();it_v++){
			lf_veh = it_v->second;
			delete lf_veh;
		}
		delete vm_edge;
		
	}
	allVehiclesMapEdges.clear();
}

void
LaneFreeSimulationPlugin::insert_vehicle(MSVehicle* veh){
	NumericalID edge_id = veh->getLane()->getEdge().getNumericalID();
	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(edge_id);
	
	if(it==allVehiclesMapEdges.end()){
		VehicleMap* vm = new VehicleMap;
		allVehiclesMapEdges.insert(std::make_pair(edge_id,vm));
		it = allVehiclesMapEdges.find(edge_id);
	}
	
	VehicleMap* vm = it->second;


	MSLaneFreeVehicle* new_veh = new MSLaneFreeVehicle(veh);
	NumericalID veh_nid = veh->getNumericalID();
	vm->insert(std::make_pair(veh_nid,new_veh));
	double lv = veh->getVehicleType().getLength();
	if(lv>max_vehicle_length){
		max_vehicle_length = lv;
	}

	InsertedLateralInitStatus::iterator it_l = insertedLatInitStatus.find(veh_nid);
	if(it_l!=insertedLatInitStatus.end()){
		double pos_y = (it_l->second)[0];
		double speed_y = (it_l->second)[1];
		new_veh->set_position_y(pos_y);
		new_veh->set_speed_y(speed_y);
		insertedLatInitStatus.erase(veh_nid);
	}
	// double init_pos_x=new_veh->get_position_x(), init_pos_y=new_veh->get_position_y(), init_speed_x=new_veh->get_speed_x();
	
	event_vehicle_enter(veh_nid);
	// new_veh->set_position_x(init_pos_x);
	// new_veh->set_position_y(init_pos_y);
	// new_veh->set_speed_x(init_speed_x);
	// std::cout << "Vehicle "<< veh->getNumericalID() <<" inserted!\n";
	

}

MSLaneFreeVehicle*
LaneFreeSimulationPlugin::find_vehicle(NumericalID veh_id){
	
	VehicleMap* vm;
	VehicleMap::iterator it_v;

	for(VehicleMapEdges::iterator it = allVehiclesMapEdges.begin();it!=allVehiclesMapEdges.end();it++){
		vm = it->second;
		it_v =  vm->find(veh_id);
		if(it_v!=vm->end()){
			return it_v->second;
		}

	}
	

	
	std::cout << "Vehicle "<< veh_id <<" not found in any edge!\n";
	return nullptr;
	

}


NumericalID
LaneFreeSimulationPlugin::find_edge(NumericalID veh_id){
	
	VehicleMap* vm;
	VehicleMap::iterator it_v;

	for(VehicleMapEdges::iterator it = allVehiclesMapEdges.begin();it!=allVehiclesMapEdges.end();it++){
		vm = it->second;
		it_v =  vm->find(veh_id);
		if(it_v!=vm->end()){
			return it->first;
		}

	}
	

	
	std::cout << "Vehicle "<< veh_id <<" not found in any edge!\n";
	return -1;
	

}


MSLaneFreeVehicle*
LaneFreeSimulationPlugin::find_vehicle_in_edge(NumericalID veh_id, NumericalID edge_id){
	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(edge_id);
	
	if(it==allVehiclesMapEdges.end()){
		std::cout << "Edge "<< edge_id <<" not found!\n";
		return nullptr;
	}

	VehicleMap* vm = it->second;

	VehicleMap::iterator it_v =  vm->find(veh_id);

	if(it_v==vm->end()){
		std::cout << "Vehicle "<< veh_id <<" not found in edge"<< edge_id <<"!\n";
		return nullptr;
	}

	return it_v->second;

}



NumericalID
LaneFreeSimulationPlugin::find_stored_edge(MSVehicle* veh){
	
	for(VehicleMapEdges::iterator it=allVehiclesMapEdges.begin();it!=allVehiclesMapEdges.end();it++){
		if((it->second)->find(veh->getNumericalID())!=(it->second)->end()){
			return it->first;
		}

	}

	std::cout << "Vehicle "<< veh->getNumericalID() <<" not found in any edge.\n";
	return -1;
}

void LaneFreeSimulationPlugin::change_edge(MSVehicle* veh){
	NumericalID new_edge_id = veh->getLane()->getEdge().getNumericalID();
	NumericalID old_edge_id = find_stored_edge(veh);
	if(old_edge_id==new_edge_id){
		return;
	}
	
	VehicleMap* old_edge_vm = allVehiclesMapEdges[old_edge_id];
	
		

	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(new_edge_id);
	
	if(it==allVehiclesMapEdges.end()){		
		VehicleMap* new_edge = new VehicleMap;
		allVehiclesMapEdges.insert(std::make_pair(new_edge_id,new_edge));		
	}

	VehicleMap* new_edge_vm = allVehiclesMapEdges[new_edge_id];
	MSLaneFreeVehicle* ch_veh = (*old_edge_vm)[veh->getNumericalID()];
	new_edge_vm->insert(std::make_pair(veh->getNumericalID(),ch_veh));

	old_edge_vm->erase(veh->getNumericalID());

	// std::cout << "Vehicle "<< veh->getNumericalID() <<" changed from edge " << old_edge_id <<" to edge " << new_edge_id<<" !\n";

	
}

void LaneFreeSimulationPlugin::remove_vehicle(MSVehicle* veh){

	NumericalID edge_id = veh->getLane()->getEdge().getNumericalID();
	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(edge_id);
	
	if(it==allVehiclesMapEdges.end()){		
		std::cout << "Edge " << edge_id << " not found!\n";
		return;
	}
	event_vehicle_exit(veh->getNumericalID());
	VehicleMap* vm = it->second;
	MSLaneFreeVehicle* ch_veh = (*vm)[veh->getNumericalID()];
	delete ch_veh;
	vm->erase(veh->getNumericalID());
	// std::cout << "Vehicle "<< veh->getID() <<" exited the network!\n";
}

LaneFreeSimulationPlugin*
LaneFreeSimulationPlugin::getInstance(void) {
    if (myInstance != nullptr) {
        return myInstance;
    }
    throw ProcessError("The laneFreePlugin instance was not yet constructed.");
}


double MSLaneFreeVehicle::last_init_pos_y=0;
double MSLaneFreeVehicle::last_v_width=0;

//#define DEBUG_V

// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_LaneFree::MSCFModel_LaneFree(const MSVehicleType* vtype, bool idmm) :
    MSCFModel(vtype),
    myIDMM(idmm),
    myDelta(idmm ? 4.0 : vtype->getParameter().getCFParam(SUMO_ATTR_CF_IDM_DELTA, 4.)),
    myAdaptationFactor(idmm ? vtype->getParameter().getCFParam(SUMO_ATTR_CF_IDMM_ADAPT_FACTOR, 1.8) : 1.0),
    myAdaptationTime(idmm ? vtype->getParameter().getCFParam(SUMO_ATTR_CF_IDMM_ADAPT_TIME, 600.0) : 0.0),
    myIterations(MAX2(1, int(TS / vtype->getParameter().getCFParam(SUMO_ATTR_CF_IDM_STEPPING, .25) + .5))),
    myTwoSqrtAccelDecel(double(2 * sqrt(myAccel * myDecel))) {
    // IDM does not drive very precise and may violate minGap on occasion
    myCollisionMinGapFactor = vtype->getParameter().getCFParam(SUMO_ATTR_COLLISION_MINGAP_FACTOR, 0.5);


    // if(!(LaneFreeSimulationPlugin::hasInstance())){
    	
    // 	new LaneFreeSimulationPlugin();
    	
    // }
}

MSCFModel_LaneFree::~MSCFModel_LaneFree() {}



double
MSCFModel_LaneFree::finalizeSpeed(MSVehicle* const veh, double vPos) const {
    double vNext = MSCFModel::finalizeSpeed(veh, vPos);
    if (myAdaptationFactor != 1.) {
        VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
        vars->levelOfService += (vNext / veh->getLane()->getVehicleMaxSpeed(veh) - vars->levelOfService) / myAdaptationTime * TS;
    }

    // if(vNext>25){
    // 	vNext = 25;
    // }
    // vNext = 25;
    // double v_lf_model = LaneFreeSimulationPlugin::getInstance()->get_veh_speed(veh->getNumericalID());
    // MSLaneFreeVehicle *lfveh =  LaneFreeSimulationPlugin::getInstance()->find_vehicle_in_edge(veh->getLane()->getEdge().getNumericalID(),veh->getNumericalID());
    MSLaneFreeVehicle *lfveh =  LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh->getNumericalID());
    if(lfveh==nullptr){
    	std::cout<<"Issue\n";
    	return vNext;
    }
    
    
    vNext = lfveh->apply_acceleration_internal();
    
    // std::cout<<vNext<<"\n";
    // long long int vid_i = veh->getNumericalID();
    // std::string vid = std::to_string(vid_i);
    // veh->setID(vid);
    // Position vcpos = veh->getPosition();
    // MSEdge *vedge = &(veh->getLane())->getEdge();
    
    // double posy = lfveh->get_position_y();

    // std::cout << "vid:" << veh->getID() << " at lateral pos: "<< posy<<"\n";        
    // std::cout << "vid num: " << veh->getNumericalID() << "\n";
    // std::cout << "vlatPos: " <<std::setprecision(4)<< veh->getLateralPositionOnLane() << "\n";
    // std::cout << "vlongPos: " <<std::setprecision(4)<< veh->getPositionOnLane() << "\n";
    // std::cout << "vlane: " << veh->getLane()->getID() << "\n";
    // std::cout << "vedge: " << (vedge)->getNumericalID() << "\n";
    // std::cout << "vedge dist: " << (vedge)->getDistance() << "\n";
    // std::cout << "v global pos: x:" << vcpos.x() << "y:" << vcpos.y() << "\n";        
    // veh->setLateralPositionOnLane(veh->getLateralPositionOnLane()+0.01);
    // Student s("Joe");
    // s.display();
    // foo_example_2 = &foo_example_2_t;
    // double r_ex = foo_example(4);
    // std::cout << "foo func: " << r_ex << "\n";
    // veh->setLateralPositionOnLane(veh->getLateralPositionOnLane()+0.001);
    
    return vNext;
}


double
MSCFModel_LaneFree::freeSpeed(const MSVehicle* const veh, double speed, double seen, double maxSpeed, const bool /*onInsertion*/) const {
    if (maxSpeed < 0.) {
        // can occur for ballistic update (in context of driving at red light)
        return maxSpeed;
    }
    const double secGap = getSecureGap(veh, nullptr, maxSpeed, 0, myDecel);
    double vSafe;
    if (speed <= maxSpeed) {
        // accelerate
        vSafe = _v(veh, 1e6, speed, maxSpeed, veh->getLane()->getVehicleMaxSpeed(veh), false);
    } else {
        // decelerate
        // @note relax gap to avoid emergency braking
        // @note since the transition point does not move we set the leader speed to 0
        vSafe = _v(veh, MAX2(seen, secGap), speed, 0, veh->getLane()->getVehicleMaxSpeed(veh), false);
    }
    if (seen < secGap) {
        // avoid overshoot when close to change in speed limit
        vSafe = MIN2(vSafe, maxSpeed);
    }
    //std::cout << SIMTIME << " speed=" << speed << " maxSpeed=" << maxSpeed << " seen=" << seen << " secGap=" << secGap << " vSafe=" << vSafe << "\n";
    return vSafe;
}


double
MSCFModel_LaneFree::followSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const pred) const {
    applyHeadwayAndSpeedDifferencePerceptionErrors(veh, speed, gap2pred, predSpeed, predMaxDecel, pred);
#ifdef DEBUG_V
    gDebugFlag1 = veh->isSelected();
#endif
    return _v(veh, gap2pred, speed, predSpeed, veh->getLane()->getVehicleMaxSpeed(veh));
}


double
MSCFModel_LaneFree::insertionFollowSpeed(const MSVehicle* const v, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const /*pred*/) const {
    // see definition of s in _v()
    double s = MAX2(0., speed * myHeadwayTime + speed * (speed - predSpeed) / myTwoSqrtAccelDecel);
    if (gap2pred >= s) {
        // followSpeed always stays below speed because s*s / (gap2pred * gap2pred) > 0. This would prevent insertion with maximum speed at all distances
        return speed;
    } else {
        return followSpeed(v, speed, gap2pred, predSpeed, predMaxDecel);
    }
}


double
MSCFModel_LaneFree::stopSpeed(const MSVehicle* const veh, const double speed, double gap) const {
    applyHeadwayPerceptionError(veh, speed, gap);
    if (gap < 0.01) {
        return 0;
    }
    double result = _v(veh, gap, speed, 0, veh->getLane()->getVehicleMaxSpeed(veh));
    if (gap > 0 && speed < NUMERICAL_EPS && result < NUMERICAL_EPS) {
        // ensure that stops can be reached:
        //std::cout << " switching to krauss: " << veh->getID() << " gap=" << gap << " speed=" << speed << " res1=" << result << " res2=" << maximumSafeStopSpeed(gap, speed, false, veh->getActionStepLengthSecs())<< "\n";
        result = maximumSafeStopSpeed(gap, speed, false, veh->getActionStepLengthSecs());
    }
    //if (result * TS > gap) {
    //    std::cout << "Maximum stop speed exceeded for gap=" << gap << " result=" << result << " veh=" << veh->getID() << " speed=" << speed << " t=" << SIMTIME << "\n";
    //}
    return result;
}


/// @todo update interactionGap logic to IDM
double
MSCFModel_LaneFree::interactionGap(const MSVehicle* const veh, double vL) const {
    // Resolve the IDM equation to gap. Assume predecessor has
    // speed != 0 and that vsafe will be the current speed plus acceleration,
    // i.e that with this gap there will be no interaction.
    const double acc = myAccel * (1. - pow(veh->getSpeed() / veh->getLane()->getVehicleMaxSpeed(veh), myDelta));
    const double vNext = veh->getSpeed() + acc;
    const double gap = (vNext - vL) * (veh->getSpeed() + vL) / (2 * myDecel) + vL;

    // Don't allow timeHeadWay < deltaT situations.
    return MAX2(gap, SPEED2DIST(vNext));
}

double
MSCFModel_LaneFree::getSecureGap(const MSVehicle* const /*veh*/, const MSVehicle* const /*pred*/, const double speed, const double leaderSpeed, const double /*leaderMaxDecel*/) const {
    const double delta_v = speed - leaderSpeed;
    return MAX2(0.0, speed * myHeadwayTime + speed * delta_v / myTwoSqrtAccelDecel);
}


double
MSCFModel_LaneFree::_v(const MSVehicle* const veh, const double gap2pred, const double egoSpeed,
                  const double predSpeed, const double desSpeed, const bool respectMinGap) const {
// this is more or less based on http://www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html
// and http://arxiv.org/abs/cond-mat/0304337
// we assume however constant speed for the leader
    double headwayTime = myHeadwayTime;
    if (myAdaptationFactor != 1.) {
        const VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
        headwayTime *= myAdaptationFactor + vars->levelOfService * (1. - myAdaptationFactor);
    }
    double newSpeed = egoSpeed;
    double gap = gap2pred;
    if (respectMinGap) {
        // gap2pred comes with minGap already subtracted so we need to add it here again
        gap += myType->getMinGap();
    }
    for (int i = 0; i < myIterations; i++) {
        const double delta_v = newSpeed - predSpeed;
        double s = MAX2(0., newSpeed * headwayTime + newSpeed * delta_v / myTwoSqrtAccelDecel);
        if (respectMinGap) {
            s += myType->getMinGap();
        }
        gap = MAX2(NUMERICAL_EPS, gap); // avoid singularity
        const double acc = myAccel * (1. - pow(newSpeed / desSpeed, myDelta) - (s * s) / (gap * gap));
#ifdef DEBUG_V
        if (gDebugFlag1) {
            std::cout << " gap=" << gap << " t=" << myHeadwayTime << " t2=" << headwayTime << " s=" << s << " pow=" << pow(newSpeed / desSpeed, myDelta) << " gapDecel=" << (s * s) / (gap * gap) << " a=" << acc;
        }
#endif
        newSpeed = MAX2(0.0, newSpeed + ACCEL2SPEED(acc) / myIterations);
#ifdef DEBUG_V
        if (gDebugFlag1) {
            std::cout << " v2=" << newSpeed << "\n";
        }
#endif
        //TODO use more realistic position update which takes accelerated motion into account
        gap -= MAX2(0., SPEED2DIST(newSpeed - predSpeed) / myIterations);
    }
    return MAX2(0., newSpeed);
}


MSCFModel*
MSCFModel_LaneFree::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_LaneFree(vtype, myIDMM);
}
