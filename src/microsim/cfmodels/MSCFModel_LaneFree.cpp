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
#include <microsim/MSRoute.h>
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
		void* tmp_ptr = s->ptr;
		s->ptr = realloc(s->ptr, allocated_blocks * block_size);
		if (s->ptr == NULL) {
			free(tmp_ptr);
		}
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
	// TODO: check more efficient way to obtain edge object from edge_id (check whether the edges_v[edge_id] works, from MSEdge::dictionary(string,ptr))
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

	//std::vector<const SUMOVehicle*> vehs = thisedge->getVehicles();
	//TODO: this sort is performed twice (once here, once for collision check)
	//std::sort(vehs.begin(), vehs.end(), less_than_key());
	//size_t veh_size = vehs.size();
	
	std::vector<const SUMOVehicle*>* vehs = LaneFreeSimulationPlugin::getInstance()->get_sorted_vehicles_in_edge(edge_id);
	arrayMemStruct* all_ids_in_edge_ams = LaneFreeSimulationPlugin::getInstance()->get_all_ids_in_edge_mem();
	if (vehs == nullptr) {
		all_ids_in_edge_ams->updated = true;
		all_ids_in_edge_ams->usize = 0;
		return NULL;
	}
	
	size_t veh_size = vehs->size();
	update_arraymemory_size(all_ids_in_edge_ams, veh_size);

	int i = 0;
	NumericalID* all_ids_in_edge_array = (NumericalID*)	all_ids_in_edge_ams->ptr;
	if(all_ids_in_edge_array != NULL){
		for (const SUMOVehicle* veh : *vehs){
			
			all_ids_in_edge_array[i] = veh->getNumericalID();
			i++;
			
		}
	}

	all_ids_in_edge_ams->updated = true;
	all_ids_in_edge_ams->usize = veh_size;
	return all_ids_in_edge_array;




}

// get the length of a given road
double lf_plugin_get_edge_length(NumericalID edge_id) {
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();

	bool found = false;
	MSEdge* fedge = nullptr;
	for (MSEdge* edge : edges_v) {
		if (edge->getNumericalID() == edge_id) {
			found = true;
			fedge = edge;
			break;
		}

	}

	if (!found) {
		std::cout << "Edge with id " << edge_id << "not found!\n";
		return -1;
	}




	return fedge->getLength();

}


// get the width of a given road
double lf_plugin_get_edge_width(NumericalID edge_id) {
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();

	bool found = false;
	MSEdge* fedge = nullptr;
	for (MSEdge* edge : edges_v) {
		if (edge->getNumericalID() == edge_id) {
			found = true;
			fedge = edge;
			break;
		}

	}

	if (!found) {
		std::cout << "Edge with id " << edge_id << "not found!\n";
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
double lf_plugin_get_veh_length(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_vehicle()->getVehicleType().getLength();
}

// get the width of the veh_id vehicle
double lf_plugin_get_veh_width(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_vehicle()->getVehicleType().getWidth();

}

bool eq_vehicle(const SUMOVehicle* v, const NumericalID veh_id) {
	bool equal = (v->getNumericalID() == veh_id);	
	return equal;
}
//code from https://stackoverflow.com/a/46824354
size_t binary_search_find_index_d(SortedVehiclesVector* sorted_vehs, NumericalID veh_id) {
	SortedVehiclesVector::iterator it = std::lower_bound(sorted_vehs->begin(), sorted_vehs->end(), veh_id, eq_vehicle);
	std::cout << (*it)->getNumericalID() << " " << veh_id << " qeqe\n";

	if (it == sorted_vehs->end() || (*it)->getNumericalID() != veh_id) {
		return -1;
	}
	else {
		size_t index = std::distance(sorted_vehs->begin(), it);
		return index;
	}
}


//code based on https://www.geeksforgeeks.org/binary-search/
size_t binary_search_find_index (SortedVehiclesVector* sorted_vehs, int start, int end, NumericalID veh_id, double pos_x)
{
	//std::cout << "start " << start << " end " << end << "\n";
	if (end >= start) {
		int mid = start + (end - start) / 2;
		//std::cout << "mid " << mid <<"\n";
		// If the element is present at the middle 
		// itself 
		NumericalID mid_id = sorted_vehs->at(mid)->getNumericalID();
		if (mid_id == veh_id)
			return (size_t)mid;

		// If element is smaller than mid, then 
		// it can only be present in left subarray 
		double pos_mid = get_position_x(mid_id);
		if (pos_mid > pos_x) {
			return binary_search_find_index(sorted_vehs, start, mid - 1, veh_id, pos_x);
		}
		if (pos_mid == pos_x) {
			
			NumericalID tmp_id;
			int i;
			i = mid - 1;
			//std::cout <<"while "<< i <<">= "<< start<< " "<< (i >= start)<<"\n";
			while (i >= start) {
				//std::cout << i << "\n";
				tmp_id = sorted_vehs->at(i)->getNumericalID();
				if (tmp_id == veh_id) {
					return i;

				}
				i--;
			}
			
			i = mid + 1;
			while (i <= end) {
				//std::cout << i << "\n";
				tmp_id = sorted_vehs->at(i)->getNumericalID();
				if (tmp_id == veh_id) {
					return i;

				}
				i++;
			}

			std::cout << "Something wrong with binary search!\n";
			return -1;
		}
		// Else the element can only be present 
		// in right subarray 
		return binary_search_find_index(sorted_vehs, mid + 1, end, veh_id, pos_x);
	}
	// We reach here when element is not 
	// present in array 
	return -1;
}


//cross_edge: whether to get ids beyond the vehicle's current edge (1:yes, 0:no)
//we need to have access to an ordered list of all vehicles based on the current edge
// this is already performed, for collision check. So, we should sort the vehicles, and keep this info on a shared data structure
// Also, gain access to relevant info, the next/previous edge of veh_id, based on its route. Moreover, we may need access to edges 
// that are not in the veh's route, i.e., in an on-ramp do we care for vehicles inside the motorway, before the merging?
// And should it be in this function, or in a separate one?
NumericalID* lf_plugin_get_all_neighbor_ids_front(NumericalID veh_id, double front_distance, int cross_edge, size_t* neighbors_size) {	
	
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	
	// Assert that requested vehicle exists
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return NULL;
	}

	// Get route object of vehicle
	const MSRoute veh_route = lfveh->get_vehicle()->getRoute();
	
	// This returns the set of edges corresponding to the vehicle's path, including the internal ones (on junctions)
	const ConstMSEdgeVector veh_edges = veh_route.getEdgeswInternal();

	// local position of vehicle
	double x_vid = lfveh->get_position_x();

	// current edge id
	NumericalID edge_id = lfveh->get_vehicle()->getLane()->getEdge().getNumericalID();//this will contain the edge, also accounting for intersection
	
	// find the current edge's index on the route array
	int my_edge_index{ veh_route.edge_index(&(lfveh->get_vehicle()->getLane()->getEdge())) };
	
	// check whether edge was found
	if (my_edge_index == -1) {
		// last edge object pointer
		const MSEdge* last_edge = veh_edges.at(veh_edges.size() - 1);
		//this is an error only if the vehicle is not in the last edge already
		if (last_edge->getNumericalID() != edge_id) {
			std::cout << "Edge " << lfveh->get_vehicle()->getLane()->getEdge().getID() << " not found in route for vehicle " << lfveh->get_vehicle()->getID() << "!\n";
			
		}

		// if it is, then it means that the vehicle just exited
		// TODO check this again!
		*neighbors_size = 0;
		return NULL;

	}

	// this will be filled with the following edges starting from the current one
	ConstMSEdgeVector all_veh_edges; //will contain the internal edges as well
	for (int i = my_edge_index; i < veh_edges.size(); i++) {
		all_veh_edges.push_back(veh_edges.at(i));
	}
	
	
	// all vehicles in current edge, sorted according to (local) longitudinal position
	SortedVehiclesVector* sorted_vehs = LaneFreeSimulationPlugin::getInstance()->get_sorted_vehicles_in_edge(edge_id);
	
	size_t size_edge = sorted_vehs->size();
	if (size_edge == 0) {
		std::cout << "ERROR: Sorted edge of vehicle found empty!\n"; // current edge should have at least ego veh_id
		*neighbors_size = 0;
		return NULL;
	}

	size_t found = binary_search_find_index(sorted_vehs, 0, (int)(size_edge-1), veh_id, x_vid);
	
	if (found == -1) {
		
		std::cout << "\nVehicle " << lfveh->get_vehicle()->getID() << " not found in sorted vector of edge " << all_veh_edges[my_edge_index]->getID() << "!\n";
		*neighbors_size = 0;
		return NULL;
	}
	std::vector<std::pair<double, MSVehicle*>> neighbors_with_distance;
	LaneFreeSimulationPlugin::getInstance()->get_all_neighbors_internal(lfveh, veh_edges.at(my_edge_index), sorted_vehs, found, front_distance, true, cross_edge, neighbors_with_distance);


	

	// update the allocated memory if needed
	arrayMemStruct* all_neighbor_ids_front_ams = LaneFreeSimulationPlugin::getInstance()->get_all_neighbor_ids_front_mem();

	size_t n_size = neighbors_with_distance.size();
	update_arraymemory_size(all_neighbor_ids_front_ams, n_size);

	// construct the resulting array
	int i = 0;
	NumericalID* all_neighbor_ids_front_array = (NumericalID*)all_neighbor_ids_front_ams->ptr;
	if (all_neighbor_ids_front_array != NULL) {
		for (std::pair<double, MSVehicle*> nb : neighbors_with_distance) {
			
			all_neighbor_ids_front_array[i] = nb.second->getNumericalID();
			/*if (lfveh->get_vehicle()->getID() == "normal_flow_7_1.2") {
				std::cout << nb.second->getID() << "\n";
			}*/
			i++;

		}
	}

	all_neighbor_ids_front_ams->updated = true;
	all_neighbor_ids_front_ams->usize = n_size;

	*neighbors_size = n_size;
	return all_neighbor_ids_front_array;
	
}


//cross_edge: whether to get ids beyond the vehicle's current edge (1:yes, 0:no)
//we need to have access to an ordered list of all vehicles based on the current edge
// this is already performed, for collision check. So, we should sort the vehicles, and keep this info on a shared data structure
// Also, gain access to relevant info, the next/previous edge of veh_id, based on its route. Moreover, we may need access to edges 
// that are not in the veh's route, i.e., in an on-ramp do we care for vehicles inside the motorway, before the merging?
// And should it be in this function, or in a separate one?
NumericalID* lf_plugin_get_all_neighbor_ids_back(NumericalID veh_id, double back_distance, int cross_edge, size_t* neighbors_size) {

	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);

	// Assert that requested vehicle exists
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return NULL;
	}

	// Get route object of vehicle
	const MSRoute veh_route = lfveh->get_vehicle()->getRoute();

	// This returns the set of edges corresponding to the vehicle's path, including the internal ones (on junctions)
	const ConstMSEdgeVector veh_edges = veh_route.getEdgeswInternal();

	// local position of vehicle
	double x_vid = lfveh->get_position_x();

	// current edge id
	NumericalID edge_id = lfveh->get_vehicle()->getLane()->getEdge().getNumericalID();//this will contain the edge, also accounting for intersection

	// find the current edge's index on the route array
	int my_edge_index{ veh_route.edge_index(&(lfveh->get_vehicle()->getLane()->getEdge())) };

	// check whether edge was found
	if (my_edge_index == -1) {
		// last edge object pointer
		const MSEdge* last_edge = veh_edges.at(veh_edges.size() - 1);
		//this is an error only if the vehicle is not in the last edge already
		if (last_edge->getNumericalID() != edge_id) {
			std::cout << "Edge " << lfveh->get_vehicle()->getLane()->getEdge().getID() << " not found in route for vehicle " << lfveh->get_vehicle()->getID() << "!\n";

		}

		// if it is, then it means that the vehicle just exited
		// TODO check this again!
		*neighbors_size = 0;
		return NULL;

	}

	// this will be filled with the following edges starting from the current one
	ConstMSEdgeVector all_veh_edges; //will contain the internal edges as well
	for (int i = my_edge_index; i < veh_edges.size(); i++) {
		all_veh_edges.push_back(veh_edges.at(i));
	}


	// all vehicles in current edge, sorted according to (local) longitudinal position
	SortedVehiclesVector* sorted_vehs = LaneFreeSimulationPlugin::getInstance()->get_sorted_vehicles_in_edge(edge_id);

	size_t size_edge = sorted_vehs->size();
	if (size_edge == 0) {
		std::cout << "ERROR: Sorted edge of vehicle found empty!\n"; // current edge should have at least ego veh_id
		*neighbors_size = 0;
		return NULL;
	}

	size_t found = binary_search_find_index(sorted_vehs, 0, (int)(size_edge - 1), veh_id, x_vid);

	if (found == -1) {

		std::cout << "\nVehicle " << lfveh->get_vehicle()->getID() << " not found in sorted vector of edge " << all_veh_edges[my_edge_index]->getID() << "!\n";
		*neighbors_size = 0;
		return NULL;
	}
	std::vector<std::pair<double, MSVehicle*>> neighbors_with_distance;
	LaneFreeSimulationPlugin::getInstance()->get_all_neighbors_internal(lfveh, veh_edges.at(my_edge_index), sorted_vehs, found, back_distance, false, cross_edge, neighbors_with_distance);




	// update the allocated memory if needed
	arrayMemStruct* all_neighbor_ids_back_ams = LaneFreeSimulationPlugin::getInstance()->get_all_neighbor_ids_back_mem();

	size_t n_size = neighbors_with_distance.size();
	update_arraymemory_size(all_neighbor_ids_back_ams, n_size);

	// construct the resulting array
	int i = 0;
	NumericalID* all_neighbor_ids_back_array = (NumericalID*)all_neighbor_ids_back_ams->ptr;
	if (all_neighbor_ids_back_array != NULL) {
		for (std::pair<double, MSVehicle*> nb : neighbors_with_distance) {

			all_neighbor_ids_back_array[i] = nb.second->getNumericalID();
			i++;

		}
	}

	all_neighbor_ids_back_ams->updated = true;
	all_neighbor_ids_back_ams->usize = n_size;


	*neighbors_size = n_size;
	return all_neighbor_ids_back_array;
	

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
		std::cout << "Ego with Numerical ID:"<< ego_id <<" not found!\n";
		return -1;
	}
	MSLaneFreeVehicle* other_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(other_id);
	if (other_lfveh== nullptr) {
		std::cout << "Other with Numerical ID:" << other_id << " not found!\n";
		return -1;
	}
	NumericalID ego_edge_id = ego_lfveh->get_vehicle()->getLane()->getEdge().getNumericalID(), other_edge_id = other_lfveh->get_vehicle()->getLane()->getEdge().getNumericalID();
	if (ego_edge_id != other_edge_id) {
		//find route of ego, and check whether the other edge is on its path
		
		const MSRoute ego_route = ego_lfveh->get_vehicle()->getRoute();
		const MSEdge* ego_edge = &ego_lfveh->get_vehicle()->getLane()->getEdge();
		const MSEdge* other_edge = &other_lfveh->get_vehicle()->getLane()->getEdge();
		double pos_ego = ego_lfveh->get_position_x();
		double pos_other = other_lfveh->get_position_x();
				
		int ego_index = ego_route.edge_index(ego_edge);
		if (ego_index == -1) {
			std::cout << "Error! edge not found!\n";
			return -1;
		}
		int other_index = ego_route.edge_index(other_edge);
		if (other_index == -1) {
			std::cout << "Error! other vehicle ("<< other_lfveh->get_vehicle()->getID() <<") with route "<< other_lfveh->get_vehicle()->getRoute().getID()<<" not in the path of ego ("<< ego_lfveh->get_vehicle()->getID() <<") with route "<< ego_lfveh->get_vehicle()->getRoute().getID() <<"!\n";
			return -1;
		}
		
		double distance;
		if (ego_index < other_index) {

			distance = ego_route.getDistanceBetween(pos_ego, pos_other, ego_edge, other_edge);//TODO include later on the routeposition that will be stored
			return distance;
		}
		else if (ego_index > other_index) {
			distance = ego_route.getDistanceBetween(pos_other, pos_ego, other_edge, ego_edge);//TODO include later on the routeposition that will be stored
			return -distance;
		}
		else {
			std::cout << "Error! edge indices should not be the same";
			return -1;
		}
		/*
		const ConstMSEdgeVector veh_edges = veh_route.getEdges();
		double x_vid = ego_lfveh->get_position_x();
		

		ConstMSEdgeVector all_veh_edges; //contains also the internal edges
		NumericalID edge_id = ego_edge_id;
		
		size_t route_edge_index = 0;//-1;
		const MSEdge* tmp_edge = veh_edges.at(0);
		const MSEdge* tmp_edge_next;
		const MSEdge* tmp_internal_edge;

		

		
		
		//std::cout << "Route of " << lfveh->get_vehicle()->getID()<<", with current edge:"<< lfveh->get_vehicle()->getLane()->getEdge().getID() <<", reg:"<< lfveh->get_vehicle()->getEdge()->getID()<<", and position:"<< lfveh->get_position_x() <<" :";
		for (size_t i = 0; i < veh_edges.size() - 1; i++) {//calculate the distance inside here, or maybe try the function provided, which may give directly the answer getDistanceBetween
			tmp_edge_next = veh_edges.at(i + 1);
			tmp_internal_edge = tmp_edge->getInternalFollowingEdge(tmp_edge_next);

			if (found_edge) {
				all_veh_edges.push_back(tmp_edge);
				all_veh_edges.push_back(tmp_internal_edge);
				//std::cout << " "<<tmp_edge->getID()<<" "<<tmp_internal_edge->getID();
			}
			else if (tmp_edge->getNumericalID() == edge_id) {
				found_edge = true;
				all_veh_edges.push_back(tmp_edge);
				all_veh_edges.push_back(tmp_internal_edge);
			}
			else if (tmp_internal_edge->getNumericalID() == edge_id) {
				found_edge = true;
				all_veh_edges.push_back(tmp_internal_edge);
			}

			if (tmp_edge->getNumericalID() == other_edge_id || tmp_internal_edge->getNumericalID() == edge_id) {
				found_other = true;
			}
			tmp_edge = tmp_edge_next;
		}

		if (tmp_edge->getNumericalID() == other_edge_id) {
			found_other = true;
		}

		if (!found_other) {
			std::cout << "Edge of other vehicle not found!\n";
		}
		all_veh_edges.push_back(tmp_edge);
		//std::cout << " " << tmp_edge->getID() << " ";
		//std::cout << "\n";
		if (!found_edge) {
			//this is an error only if the vehicle is not in the last edge already
			if (tmp_edge->getNumericalID() != edge_id) {
				std::cout << "Edge not found in route for vehicle " << ego_lfveh->get_vehicle()->getID() << "!\n";
			}

		}
		
		size_t route_ego_edge_index = -1, route_other_edge_index = -1;
		double total_length = 0;
		double edge_length = lf_plugin_get_edge_length(ego_edge_id);
		NumericalID current_edge_id;
		bool found_ego_edge = false;
		for (size_t i = 0; i < veh_edges.size(); i++) {
			current_edge_id = veh_edges[i]->getNumericalID();
			if (current_edge_id == ego_edge_id) {
				route_ego_edge_index = i;
				found_ego_edge = true;
			}

			if (veh_edges[i]->getNumericalID() == route_other_edge_index) {
				route_other_edge_index = i;
				break;
			}
			
			if (found_ego_edge) {
				total_length = total_length + lf_plugin_get_edge_length(current_edge_id);
			}
			

			
		}

		if (route_other_edge_index < route_ego_edge_index) {
			std::cout<< "Vehicles are not on the same edge! Vehicles upstream of ego are not currently supported!\n";
			return -1;
		}
		if (route_other_edge_index == -1) {
			//std::cout << "Other vehicle not in the route of ego!\n";
			return -1;
		}

		


		double x_vid = ego_lfveh->get_position_x();

		double other_x_vid = other_lfveh->get_position_x();
		double x_relative = x_vid - total_length;


		
		return other_x_vid - x_relative;*/


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
	std::cout << "This function 'lf_plugin_get_relative_position_x' is deprecated and will be removed in later versions\n";
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
		std::cout << "Ego with Numerical ID:" << ego_id << " not found!\n";
		return -1;
	}
	MSLaneFreeVehicle* other_lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(other_id);
	if (other_lfveh == nullptr) {
		std::cout << "Other with Numerical ID:" << other_id << " not found!\n";
		return -1;
	}
	double latShift = 0, total_latShift=0;
	NumericalID ego_edge_id = ego_lfveh->get_vehicle()->getLane()->getEdge().getNumericalID(), other_edge_id = other_lfveh->get_vehicle()->getLane()->getEdge().getNumericalID();
	if (ego_edge_id != other_edge_id) {
		//find route of ego, and check whether the other edge is on its path

		const MSRoute ego_route = ego_lfveh->get_vehicle()->getRoute();
		const MSEdge* ego_edge = &ego_lfveh->get_vehicle()->getLane()->getEdge();
		const MSEdge* other_edge = &other_lfveh->get_vehicle()->getLane()->getEdge();
		double pos_ego = ego_lfveh->get_position_x();
		double pos_other = other_lfveh->get_position_x();

		int ego_index = ego_route.edge_index(ego_edge);
		if (ego_index == -1) {
			std::cout << "Error! edge not found!\n";
			return -1;
		}
		int other_index = ego_route.edge_index(other_edge);
		if (other_index == -1) {
			std::cout << "Error! other vehicle (" << other_lfveh->get_vehicle()->getID() << ") with route " << other_lfveh->get_vehicle()->getRoute().getID() << " not in the path of ego (" << ego_lfveh->get_vehicle()->getID() << ") with route " << ego_lfveh->get_vehicle()->getRoute().getID() << "!\n";
			return -1;
		}
		
		
		int start_index, end_index;
		int coeff;
		if (ego_index < other_index) {
			start_index = ego_index;
			end_index = other_index;
			coeff = -1;	
		}
		else if (ego_index > other_index) {
			start_index = other_index;
			end_index = ego_index;
			coeff = +1;
		}
		else {
			std::cout << "Error! edge indices should not be the same";
			return -1;
		}
		ConstMSEdgeVector route_edges = ego_route.getEdgeswInternal();

		const MSEdge* tmp_edge = route_edges.at(start_index);
		const MSEdge* tmp_edge_next;
		//printf("latshift:");
		for (int i = start_index +1 ; i <= end_index; i++) {
			tmp_edge_next = route_edges.at(i);
			
			latShift = tmp_edge->getLateralShiftToFollowingEdge(tmp_edge_next);
			if (latShift == 0 && i <= end_index && tmp_edge_next->isInternal()) {
				latShift = tmp_edge->getLateralShiftToFollowingEdge(route_edges.at(i + 1));				
			}
			//printf("%f\t", latShift);
			total_latShift += latShift;
			tmp_edge = tmp_edge_next;
		}
		//printf("\n");

		total_latShift = coeff * total_latShift;
		
	}

	double dy = other_lfveh->get_position_y() - ego_lfveh->get_position_y() + total_latShift;

	return dy;
}

// This code calculates the boundary's values and is based from Karteek's implementation
void boundary_value(double mid_height, std::vector<double>& lim, std::vector<double>& slope, std::vector<double>& offset, double long_pos, double veh_speed_x, double* boundary_distance, double* boundary_speed) {
	double boundary_distance_tmp = 0;
	double boundary_speed_tmp = 0;
	int size_points = lim.size();
	for (size_t i = 0; i < size_points - 1; i++) {
		if (long_pos < offset[i]) {
			boundary_distance_tmp += mid_height * (lim[i + 1] - lim[i]) * tanh(slope[i] * (long_pos - offset[i]));
			boundary_speed_tmp += mid_height * (lim[i + 1] - lim[i]) * pow(cosh(slope[i] * (long_pos - offset[i])), -2) * slope[i] * veh_speed_x;
		}
		else {
			boundary_distance_tmp += (1 - mid_height) * (lim[i + 1] - lim[i]) * tanh((mid_height / (1 - mid_height)) * slope[i] * (long_pos - offset[i]));
			boundary_speed_tmp += mid_height * slope[i] * veh_speed_x * (lim[i + 1] - lim[i]) * pow(cosh((mid_height / (1 - mid_height)) * slope[i] * (long_pos - offset[i])), -2);
		}
	}
	*boundary_distance = boundary_distance_tmp + lim[0] + mid_height * (lim[size_points - 1] - lim[0]);
	if (boundary_speed == nullptr) {
		return;
	}
	*boundary_speed = boundary_speed_tmp;
}




// calculates the lateral distance from left and right road boundaries for veh_id, at longitudinal_distance_x (vehicle can also observe upstream with negative values) and lateral_distance_y
// regarding the boundaries, it calculates the boundaries's distances (with left_boundary_distance, right_boundary_distance variables) and first derivative (with left_boundary_speed, right_boundary_speed variables)
// if speed information is not useful, one can simply place NULL pointers to the respective arguments (left_boundary_speed, right_boundary_speed)
// for speed information the argument veh_longitudinal_speed provides the longitudinal speed of the vehicle for the calculation of left_boundary_speed, right_boundary_speed. 
// if veh_longitudinal_speed is a NULL pointer, then the current speed of the vehicle will be choosed for these calculations
void lf_plugin_get_distance_to_road_boundaries_at(NumericalID veh_id, double longitudinal_distance_x, double lateral_distance_y, double* left_boundary_distance, double* right_boundary_distance, double* left_boundary_speed, double* right_boundary_speed, double* veh_longitudinal_speed) {
//void lf_plugin_get_distance_to_road_boundaries_at(NumericalID veh_id, double longitudinal_distance_x, double* left_boundary_distance, double* right_boundary_distance, double* left_boundary_speed, double* right_boundary_speed) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);

	if (left_boundary_distance == nullptr || right_boundary_distance == nullptr) {
		std::cout << "ERROR: Null pointers for boundary distance arguments!\n";
		return;
	}
	if (lfveh == nullptr) {
		std::cout << "ERROR: Vehicle not found!\n";
		return;
	}
	const MSVehicle* myveh = lfveh->get_vehicle();
	const MSRoute veh_route = myveh->getRoute();
	
	std::vector<double> leftBoundaryLevelPoints = veh_route.getLeftBoundaryLevelPoints();
	if (leftBoundaryLevelPoints.size() == 0) {
		std::cout << "ERROR: left boundary parameters not provided!\n";
		return;
	}

	std::vector<double> leftBoundarySlopes = veh_route.getLeftBoundarySlopes();

	std::vector<double> leftBoundaryOffsets = veh_route.getLeftBoundaryOffsets();

	double global_pos_x = myveh->getPosition().x() - (myveh->getLength() / 2) * cos(myveh->getAngleRelative());
	double global_pos_y = myveh->getPosition().y() - (myveh->getLength() / 2) * sin(myveh->getAngleRelative());

	double mid_height = 0.5;
	double left_boundary_y;
	
	boundary_value(mid_height, leftBoundaryLevelPoints, leftBoundarySlopes, leftBoundaryOffsets, global_pos_x + longitudinal_distance_x, lfveh->get_speed_x(), &left_boundary_y, left_boundary_speed);


	//std::cout << "Current veh "<<myveh->getID()<<" at pos:(" << global_pos_x << "," << global_pos_y << ") with boundary at long pos " << global_pos_x + longitudinal_distance_x << " is :" << left_boundary << ". Result on lateral distance is :"<< left_boundary - global_pos_y<<"\n";
	
	double sign_coeff{ cos(myveh->getAngle()) > 0 ? 1.0 : -1.0 }; // if vehicle is on the opposite side, distances should be multiplied by -1
	
	*left_boundary_distance = sign_coeff * (left_boundary_y - (global_pos_y + lateral_distance_y) );

	std::vector<double> rightBoundaryLevelPoints = veh_route.getRightBoundaryLevelPoints();
	if (rightBoundaryLevelPoints.size() == 0) {
		std::cout << "ERROR: right boundary parameters not provided!\n";
		return;
	}

	std::vector<double> rightBoundarySlopes = veh_route.getRightBoundarySlopes();

	std::vector<double> rightBoundaryOffsets = veh_route.getRightBoundaryOffsets();

	double right_boundary_y;
	
	double veh_speed;
	
	if (veh_longitudinal_speed != NULL) {
		veh_speed = *veh_longitudinal_speed;
	}
	else {
		veh_speed = lfveh->get_speed_x();
	}

	boundary_value(mid_height, rightBoundaryLevelPoints, rightBoundarySlopes, rightBoundaryOffsets, global_pos_x + longitudinal_distance_x, veh_speed, &right_boundary_y, right_boundary_speed);


	//std::cout << "Current veh "<<myveh->getID()<<" at pos:(" << global_pos_x << "," << global_pos_y << ") with boundary at long pos " << global_pos_x + longitudinal_distance_x << " is :" << left_boundary << ". Result on lateral distance is :"<< left_boundary - global_pos_y<<"\n";

	*right_boundary_distance = sign_coeff * ( (global_pos_y + lateral_distance_y) - right_boundary_y );
}



void lf_plugin_set_circular_movement(NumericalID veh_id, bool circular) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return;
	}

	lfveh->set_ring_road(circular);
}


// get the lateral position of a given vehicle (corresponds to the distance of the vehicle's center from the right road boundary)
double lf_plugin_get_position_y(NumericalID veh_id){

	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout << "Vehicle with veh id:"<< veh_id <<" not found!\n";
		return -1;
	}

	return lfveh->get_position_y();
	
}

// get the longitudinal speed of a given vehicle
double lf_plugin_get_speed_x(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_speed_x();
	
}

// get the lateral speed of a given vehicle (positive when moving towards the left boundary)
double lf_plugin_get_speed_y(NumericalID veh_id){

	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_speed_y();
	
}

// set the desired speed of a given vehicle
void lf_plugin_set_desired_speed(NumericalID veh_id, double new_d_speed){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return;
	}

	lfveh->set_desired_speed(new_d_speed);

}
// get the desired speed of a given vehicle
double lf_plugin_get_desired_speed(NumericalID veh_id){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if(lfveh==nullptr){
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_desired_speed();
	
}


// apply longitudinal & lateral acceleration on a given vehicle
void lf_plugin_apply_acceleration(NumericalID veh_id, double accel_x, double accel_y){
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return;
	}

	// report if this vehicle adheres to the bicycle model
	if (lfveh->get_vehicle()->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE) {
		std::cout << "Error! Function call apply_acceleration(veh_id, accel_x, accel_y) is not appropriate for Vehicle with id:" << lfveh->get_vehicle()->getID() << " since it adheres to the bicycle model!\n";
		return;
	}

	lfveh->apply_acceleration(accel_x, accel_y);

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


// use_global_coordinates is only relevant to the bicycle model, and will be disregarded otherwise
NumericalID lf_plugin_insert_new_vehicle(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y, double theta, int use_global_coordinates){
	
	// 
	std::string id(veh_name);
	std::string routeID(route_id);
	std::string vTypeID(type_id);
	std::string depart("now");
	std::string departLane("best");
	MSVehicleType* vehicleType = MSNet::getInstance()->getVehicleControl().getVType(vTypeID);
	double depart_pos_front=0;
	bool use_bicycle = vehicleType->getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE;
	bool use_local_coordinates = !((bool)(use_global_coordinates) && use_bicycle);
	if (use_local_coordinates) {
		depart_pos_front = pos_x + vehicleType->getLength() / 2;
		if (!use_bicycle && (bool)use_global_coordinates) {
			std::cout << "Error: Control with global coordinates is only supported for vehicles that employ the bicycle model!\n Vehicle " << veh_name << " will use local coordinates.\n";
		}
	}
	
	std::string departPos = std::to_string(depart_pos_front);
	std::string departSpeed = std::to_string(speed_x);	
	
	NumericalID new_vid = libsumo::Vehicle::addR(id, routeID, vTypeID, depart, departLane, departPos, departSpeed);
	
	LaneFreeSimulationPlugin::getInstance()->add_new_veh_additional_stats(new_vid, pos_x, pos_y, speed_y, theta, !use_local_coordinates);

	
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


int lf_plugin_get_detector_value(NumericalID detector_id) {

	MSDetectorControl* detectorControl = &MSNet::getInstance()->getDetectorControl();
	
	std::vector<int> values;
	int c_i = 0;
	int c_id = (int)detector_id;
	for (SumoXMLTag i : detectorControl->getAvailableTypes()) {
		// std::cout<<i<<"\n";
		for (auto j : detectorControl->getTypedDetectors(i)) {
			if (c_i == c_id) {
				return ((MSInductLoop*)j.second)->getVehiclesCount();
			}
			c_i++;			
		}
	}
	printf("Error, detector not found!\n");
	return -1;

}

int lf_plugin_get_detector_value_for_type(NumericalID detector_id, char* veh_type) {
	std::string veh_type_str(veh_type);
	MSDetectorControl* detectorControl = &MSNet::getInstance()->getDetectorControl();
	int count;
	std::vector<int> values;
	int c_i = 0;
	int c_id = (int)detector_id;
	for (SumoXMLTag i : detectorControl->getAvailableTypes()) {
		// std::cout<<i<<"\n";
		for (auto j : detectorControl->getTypedDetectors(i)) {
			if (c_i == c_id) {
				return ((MSInductLoop*)j.second)->getVehiclesCountForType(veh_type_str);
			}
			c_i++;
		}
	}
	printf("Error, detector not found!\n");
	return -1;

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

		for (i = 0; i < size_segments; i++) {
			dens_per_segment[i] = (dens_per_segment[i] * 1000) / segment_length;
		
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


double lf_plugin_get_average_speed_on_segment_region_on_edge(NumericalID edge_id, double segment_start, double segment_end) {

	if (segment_start >= segment_end) {
		printf("Segment start point should always be before the segment end point!\n");
		return -1;
	}

	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();

	int i, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	MSVehicle* veh;


	int density = 0;
	double sum_speed = 0;
	
	double x_v;
	double edge_length;
	for (MSEdge* edge : edges_v) {
		if (edge_id != edge->getNumericalID()) {
			continue;
		}
		edge_length = edge->getLength();
		if (segment_end > edge_length) {
			printf("Segment values from %f to %f are not within the selected road!\n", segment_start, segment_end);
			return -1;
		}
		vehs_in_edge = edge->getVehicles();
		n_v = vehs_in_edge.size();
		if (n_v == 0) {
			//std::cout<<"Edge with id "<< edge_id << " is empty!\n";
			return 0;
		}
		
		
		//TODO: we should probably find the starting vehicle here with binary search, since vehicles are ordered
		for (i = 0; i < n_v; i++) {
			veh = (MSVehicle*)vehs_in_edge[i];
			x_v = veh->getPositionOnLane();
			if (x_v >= segment_start && x_v < segment_end) {
				density++;
				sum_speed += lf_plugin_get_speed_x(veh->getNumericalID());
			}
		}

		if (density > 0) {
			return sum_speed / density;
		}
		else {
			//printf("Selected region is empty!\n");
			return 0;
		}
			
	}
	std::cout << "Edge with id " << edge_id << " not found!\n";
	return -1;
}

int lf_plugin_get_density_on_segment_region_on_edge(NumericalID edge_id, double segment_start, double segment_end) {

	if (segment_start >= segment_end) {
		printf("Segment start point should always be before the segment end point!\n");
		return -1;
	}
	
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();
	
	int i, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	MSVehicle* veh;


	int density = 0;

	double x_v;
	double edge_length;
	for (MSEdge* edge : edges_v) {
		if (edge_id != edge->getNumericalID()) {
			continue;
		}
		//std::cout << "Edge:" << edge->getID() << "\n";
		edge_length = edge->getLength();
		if (segment_end > edge_length) {
			printf("Segment values from %f to %f are not within the selected road!\n", segment_start, segment_end);
			return -1;
		}
		vehs_in_edge = edge->getVehicles();
		n_v = vehs_in_edge.size();
		if (n_v == 0) {
			//std::cout<<"Edge with id "<< edge_id << " is empty!\n";
			return 0;
		}


		//we should probably find the vehicle here with binary search, since vehicles are ordered
		for (i = 0; i < n_v; i++) {
			veh = (MSVehicle*)vehs_in_edge[i];
			x_v = veh->getPositionOnLane();
			if (x_v >= segment_start && x_v < segment_end) {
				density++;
			}
		}
		// std::cout << "Success!\n";
		return (density *1000)/ (segment_end-segment_start);
	}
	std::cout << "Edge with id " << edge_id << " not found!\n";
	return -1;
}

int lf_plugin_am_i_on_acceleration_lane(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}

	return (int)(lfveh->get_vehicle()->getLane()->isAccelLane());
}


double lf_plugin_get_average_speed_on_segment_region_on_edge_for_type(NumericalID edge_id, double segment_start, double segment_end, char* veh_type) {
	std::string veh_type_str(veh_type);
	if (segment_start >= segment_end) {
		printf("Segment start point should always be before the segment end point!\n");
		return -1;
	}

	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();

	int i, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	MSVehicle* veh;


	int density = 0;
	double sum_speed = 0;

	double x_v;
	double edge_length;
	for (MSEdge* edge : edges_v) {
		if (edge_id != edge->getNumericalID()) {
			continue;
		}
		edge_length = edge->getLength();
		if (segment_end > edge_length) {
			printf("Segment values from %f to %f are not within the selected road!\n", segment_start, segment_end);
			return -1;
		}
		vehs_in_edge = edge->getVehiclesforType(veh_type_str);
		n_v = vehs_in_edge.size();
		if (n_v == 0) {
			//std::cout<<"Edge with id "<< edge_id << " is empty!\n";
			return 0;
		}


		//TODO: we should probably find the starting vehicle here with binary search, since vehicles are ordered
		for (i = 0; i < n_v; i++) {
			veh = (MSVehicle*)vehs_in_edge[i];
			x_v = veh->getPositionOnLane();
			if (x_v >= segment_start && x_v < segment_end) {
				density++;
				sum_speed += lf_plugin_get_speed_x(veh->getNumericalID());
			}
		}

		if (density > 0) {
			return sum_speed / density;
		}
		else {
			//printf("Selected region is empty!\n");
			return 0;
		}

	}
	std::cout << "Edge with id " << edge_id << " not found!\n";
	return -1;
}


int lf_plugin_get_density_on_segment_region_on_edge_for_type(NumericalID edge_id, double segment_start, double segment_end, char* veh_type) {
	std::string veh_type_str(veh_type);
	if (segment_start >= segment_end) {
		printf("Segment start point should always be before the segment end point!\n");
		return -1;
	}

	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();

	int i, n_v;
	std::vector<const SUMOVehicle*> vehs_in_edge;
	MSVehicle* veh;


	int density = 0;

	double x_v;
	double edge_length;
	for (MSEdge* edge : edges_v) {
		if (edge_id != edge->getNumericalID()) {
			continue;
		}
		//std::cout << "Edge:" << edge->getID() << "\n";
		edge_length = edge->getLength();
		if (segment_end > edge_length) {
			printf("Segment values from %f to %f are not within the selected road!\n", segment_start, segment_end);
			return -1;
		}
		vehs_in_edge = edge->getVehiclesforType(veh_type_str);
		n_v = vehs_in_edge.size();
		if (n_v == 0) {
			//std::cout<<"Edge with id "<< edge_id << " is empty!\n";
			return 0;
		}


		//we should probably find the vehicle here with binary search, since vehicles are ordered
		for (i = 0; i < n_v; i++) {
			veh = (MSVehicle*)vehs_in_edge[i];
			x_v = veh->getPositionOnLane();
			if (x_v >= segment_start && x_v < segment_end) {
				density++;
			}
		}
		//std::cout << "Success!\n";
		return (density * 1000) / (segment_end - segment_start);
	}
	std::cout << "Edge with id " << edge_id << " not found!\n";
	return -1;
}




double lf_plugin_get_global_position_x(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}

	double global_pos_x;

	if (lfveh->get_vehicle()->getGlobalCoordinatesControl()) {
		global_pos_x = lfveh->get_vehicle()->getCachedGlobalPos().x();
	}
	else {
		global_pos_x = lfveh->get_vehicle()->getPosition().x();
	}
	
	global_pos_x = global_pos_x - (lfveh->get_vehicle()->getLength() / 2) * cos(lfveh->get_vehicle()->getAngleRelative());
	return global_pos_x;
}


double lf_plugin_get_global_position_y(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}

	double global_pos_y = lfveh->get_vehicle()->getCachedGlobalPos().y() - (lfveh->get_vehicle()->getLength() / 2) * sin(lfveh->get_vehicle()->getAngleRelative());
	if (lfveh->get_vehicle()->getGlobalCoordinatesControl()) {
		global_pos_y = lfveh->get_vehicle()->getCachedGlobalPos().y();
	}
	else {
		global_pos_y = lfveh->get_vehicle()->getPosition().y();
	}
	global_pos_y = global_pos_y - (lfveh->get_vehicle()->getLength() / 2) * sin(lfveh->get_vehicle()->getAngleRelative());;
	return global_pos_y;
}


// returns the destination edge of the specific vehicle id
NumericalID lf_plugin_get_destination_edge_id(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}
	//std::cout << "Reached here for veh:" << lfveh->get_vehicle()->getID() << "\n";
	const MSRoute* veh_route = &(lfveh->get_vehicle()->getRoute());
	const ConstMSEdgeVector veh_edges = veh_route->getEdges();
	//std::cout << "Got edges\n";
	size_t size_edges = veh_edges.size();
	
	if (size_edges == 0) {
		std::cout << "Route of vehicle "<< lfveh->get_vehicle()->getID()<<" is empty!\n";
		return -1;
	}
	//std::cout << "Reached final step\n";
	const MSEdge* edge_elem = veh_edges.back();
	NumericalID res = edge_elem->getNumericalID();
	//std::cout << "return val "<<res<<"\n";
	return res;
}

// returns the origin edge of the specific vehicle id
NumericalID lf_plugin_get_origin_edge_id(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}

	const MSRoute* veh_route = &(lfveh->get_vehicle()->getRoute());
	const ConstMSEdgeVector veh_edges = veh_route->getEdges();

	size_t size_edges = veh_edges.size();

	if (size_edges == 0) {
		std::cout << "Route of vehicle " << lfveh->get_vehicle()->getID() << " is empty!\n";
		return -1;
	}

	return veh_edges.front()->getNumericalID();
}

// returns the subsequent edge id of the vehicle
NumericalID lf_plugin_get_next_edge_id(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}

	
	const MSRoute* veh_route = &(lfveh->get_vehicle()->getRoute());
	const ConstMSEdgeVector veh_edges = veh_route->getEdges();
	
	size_t size_edges = veh_edges.size();

	int my_edge_index = veh_route->edge_index_normal(lfveh->get_vehicle()->getEdge());// veh_route.edge_index(&(lfveh->get_vehicle()->getLane()->getEdge()));//
	
	if (my_edge_index == -1) {
		
		std::cout << "Edge " << lfveh->get_vehicle()->getLane()->getEdge().getID() << " not found in route for vehicle " << lfveh->get_vehicle()->getID() << "!\n";
		return -1;
	}

	if (my_edge_index == size_edges - 1) {
		return -1;
	}
	
	return veh_edges.at(my_edge_index+1)->getNumericalID();
}


// returns the previous edge id of the vehicle
NumericalID lf_plugin_get_previous_edge_id(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle not found!\n";
		return -1;
	}


	const MSRoute* veh_route = &(lfveh->get_vehicle()->getRoute());
	const ConstMSEdgeVector veh_edges = veh_route->getEdges();

	size_t size_edges = veh_edges.size();

	int my_edge_index = veh_route->edge_index_normal(lfveh->get_vehicle()->getEdge()); //veh_route.edge_index(&(lfveh->get_vehicle()->getLane()->getEdge()));//
	if (my_edge_index == -1) {

		std::cout << "Edge " << lfveh->get_vehicle()->getLane()->getEdge().getID() << " not found in route for vehicle " << lfveh->get_vehicle()->getID() << "!\n";
		return -1;
	}

	if (my_edge_index == 0) {
		return -1;
	}

	return veh_edges.at(my_edge_index-1)->getNumericalID();
}


// returns the current orientation of the vehicle, in radians with respect to the residing road
double lf_plugin_get_veh_orientation(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return -1;
	}

	return lfveh->get_vehicle()->getAngleRelative();
}



void lf_plugin_apply_control_bicycle_model(NumericalID veh_id, double F, double delta) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return;
	}

	if (lfveh->get_vehicle()->getVehicleType().getParameter().cmdModel != SUMO_TAG_LF_CMD_BICYCLE) {
		std::cout << "Vehicle with id:" << lfveh->get_vehicle()->getID() << " is nod defined to operate with the bicycle model!\n";
		return;
	}
	
	
	//std::cout << "Applied (F,delta)=("<< F<< "," << delta<<") for veh " << lfveh->get_vehicle()->getID() << "\n";
	lfveh->apply_acceleration(F, delta);

}


//return the speed of vehicle with numerical id veh_id which adheres to the bicycle model
double lf_plugin_get_speed_bicycle_model(NumericalID veh_id) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return -1;
	}

	if (lfveh->get_vehicle()->getVehicleType().getParameter().cmdModel != SUMO_TAG_LF_CMD_BICYCLE) {
		std::cout << "Vehicle with id:" << lfveh->get_vehicle()->getID() << " is nod defined to operate with the bicycle model!\n";
		return -1;
	}

	return lfveh->get_speed_x();

}


void lf_plugin_set_global_coordinate_control(NumericalID veh_id, int use_global_coordinates) {
	MSLaneFreeVehicle* lfveh = LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh_id);
	if (lfveh == nullptr) {
		std::cout << "Vehicle with veh id:" << veh_id << " not found!\n";
		return ;
	}
	
	MSVehicle* myveh = lfveh->get_vehicle();
	if (myveh->getVehicleType().getParameter().cmdModel != SUMO_TAG_LF_CMD_BICYCLE)
	{	
		std::cout << "Vehicle with veh id:" << veh_id << " does not move according to the bicycle model!\n";
	}

	myveh->setGlobalCoordinatesControl((bool)use_global_coordinates);
}
// returns the execution time (in seconds) of the previous call for the simulation_step function 
double lf_plugin_get_last_step_time() {
	return  LaneFreeSimulationPlugin::getInstance()->get_last_step_exec_time();
}

// returns the execution time (in seconds) of the previous step (disregarding the execution time for the simulation_step function, i.e., execution time for the SUMO application)
double lf_plugin_get_last_step_app_time() {
	return LaneFreeSimulationPlugin::getInstance()->get_last_step_app_exec_time();
}

double
LaneFreeSimulationPlugin::get_uniform_distribution_sample(double from, double to) {
	double val = uniform_real_dis(random_engine);

	return val * (to - from) + from;
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
	get_all_neighbor_ids_front = &lf_plugin_get_all_neighbor_ids_front;
	get_all_neighbor_ids_back = &lf_plugin_get_all_neighbor_ids_back;
	am_i_on_acceleration_lane = &lf_plugin_am_i_on_acceleration_lane;
	
	get_veh_length = &lf_plugin_get_veh_length;
	get_veh_width = &lf_plugin_get_veh_width;
	get_edge_length = &lf_plugin_get_edge_length;
	get_edge_width = &lf_plugin_get_edge_width;
	get_detector_value = &lf_plugin_get_detector_value;
	get_detector_value_for_type = &lf_plugin_get_detector_value_for_type;
	get_average_speed_on_segment_region_on_edge = &lf_plugin_get_average_speed_on_segment_region_on_edge;
	get_density_on_segment_region_on_edge = &lf_plugin_get_density_on_segment_region_on_edge;
	get_density_on_segment_region_on_edge_for_type = &lf_plugin_get_density_on_segment_region_on_edge_for_type;
	get_average_speed_on_segment_region_on_edge_for_type = &lf_plugin_get_average_speed_on_segment_region_on_edge_for_type;
	get_global_position_x = &lf_plugin_get_global_position_x;
	get_global_position_y = &lf_plugin_get_global_position_y;
	get_edge_of_vehicle = &lf_plugin_get_edge_of_vehicle;
	get_edge_name = &lf_plugin_get_edge_name;
	get_destination_edge_id = &lf_plugin_get_destination_edge_id;
	get_origin_edge_id = &lf_plugin_get_origin_edge_id;
	get_previous_edge_id = &lf_plugin_get_previous_edge_id;
	get_next_edge_id = &lf_plugin_get_next_edge_id;

	get_veh_orientation = &lf_plugin_get_veh_orientation;
	
	apply_control_bicycle_model = &lf_plugin_apply_control_bicycle_model;
	get_speed_bicycle_model = &lf_plugin_get_speed_bicycle_model;
	set_global_coordinate_control  = &lf_plugin_set_global_coordinate_control;

	get_last_step_time = &lf_plugin_get_last_step_time;
	get_last_step_app_time = &lf_plugin_get_last_step_app_time;

	get_distance_to_road_boundaries_at = &lf_plugin_get_distance_to_road_boundaries_at;

	srand(lf_plugin_get_seed());
	max_vehicle_length = 0;
	max_vehicle_width = 0;
	max_vehicle_diag = 0;
	
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

	initialise_arraymemory(&all_neighbor_ids_front, NUMID_M);
	initialise_arraymemory(&all_neighbor_ids_back, NUMID_M);

	random_engine.seed(lf_plugin_get_seed());
	uniform_real_dis = std::uniform_real_distribution<double>(0, 1);

	step_timer_seconds = -1;
	rest_app_timer_seconds = -1;;

	myInstance = this;
}

void free_mem(void* ptr) {
	if (ptr != NULL) {
		free(ptr);
	}
}

void
LaneFreeSimulationPlugin::finalize_event() {
	simulation_finalize();
}

LaneFreeSimulationPlugin::~LaneFreeSimulationPlugin() {
	
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
	free_mem(all_neighbor_ids_front.ptr);
	free_mem(all_neighbor_ids_back.ptr);
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
	lf_simulation_checkCollisions();

	before_step_time = std::chrono::steady_clock::now();
	
	if (step_timer_seconds > 0) {
		std::chrono::duration<double> dur_step(before_step_time - after_step_time);
		rest_app_timer_seconds = dur_step.count();
	}

	before_step_time = std::chrono::steady_clock::now();
	simulation_step();
	after_step_time = std::chrono::steady_clock::now();

	std::chrono::duration<double> dur_step(after_step_time - before_step_time);
	step_timer_seconds = dur_step.count();
	//printf("step time: %f s\n", step_timer_seconds);
	//printf("rest time: %f s\n", rest_app_timer_seconds);
}


void
LaneFreeSimulationPlugin::add_new_veh_additional_stats(NumericalID veh_id, double pos_x, double pos_y, double speed_y, double theta, bool use_global_coordinates){
	std::vector<double> additionals_array = {pos_y,speed_y,theta, pos_x, (double) use_global_coordinates};
	//additionals_array.push_back(pos_y);
	//additionals_array.push_back(speed_y);
	//additionals_array.push_back(theta);
	insertedAdditionalInitStatus.insert(std::make_pair(veh_id, additionals_array));
}

// we utilize an approximate procedure, where we consider the vehicle as a larger rectangle (without orientation)
// that contains inside it the oriented rectangle that is our vehicle, veh1 
// see: https://math.stackexchange.com/questions/2179500/calculating-the-dimensions-of-a-rectangle-inside-another-rectangle 
void update_vehicle_dimensions(double* length, double* width, double phi_angle) {
	// THIS FUNCTION IS NOT USED ANYMORE.
	double length_large, width_large;

	double length_tmp, width_tmp;
	
	length_tmp = *length;
	width_tmp = *width;

	length_large = sin(phi_angle) * width_tmp + cos(phi_angle) * length_tmp;
	width_large = cos(phi_angle) * width_tmp + sin(phi_angle) * length_tmp;

	*length = length_large;
	*width = width_large;
}


// given a vehicle in location (x,y) with orientation theta, and dimensions (length,width)
// it returns the 4 vertices of the rectangular through xv, yv.
// xy and yv should be already be initialized as an array of doubles with 4 elements.
void vertices(double x, double y, double theta, double length, double width, double* xv, double* yv) {
	*xv = x + (width / 2) * sin(theta);
	*(xv + 1) = x - (width / 2) * sin(theta);
	*(xv + 2) = x + length * cos(theta) - (width / 2) * sin(theta);
	*(xv + 3) = x + length * cos(theta) + (width / 2) * sin(theta);

	*yv = y - (width / 2) * cos(theta);
	*(yv + 1) = y + (width / 2) * cos(theta);
	*(yv + 2) = y + length * sin(theta) + (width / 2) * cos(theta);
	*(yv + 3) = y + length * sin(theta) - (width / 2) * cos(theta);

}



// returns 1 if point (xj,yj) lies within the rectangular formed by vehicle in location (x,y) with orientation theta, and dimensions (length,width)
int if_inside(double x, double y, double theta, double xj, double yj, double length, double width) {


	double x_t = xj - x;
	double y_t = yj - y;

	double xj_transformed = x_t * cos(theta) + y_t * sin(theta);
	double yj_transformed = y_t * cos(theta) - x_t * sin(theta);

	if (xj_transformed >= 0 && xj_transformed <= length && yj_transformed >= -width / 2 && yj_transformed <= width / 2) {
		// the point (xj,yj) lies within the rectangular area		
		return 1;
	}

	return 0;
}


// returns 1 if two vehicles collide, 0 otherwise.
int collision_check_with_orientation(double x1, double y1, double theta1, double length1, double width1, double x2, double y2, double theta2, double length2, double width2) {

	// array for vertices
	double xv[4], yv[4];

	// calcualte the vertices of vehicle 2
	vertices(x2, y2, theta2, length2, width2, xv, yv);


	int i;

	// examine whether a vertex of vehicle 2 lies within the rectangular of vehicle 1
	for (i = 0; i < 4; i++) {
		if (if_inside(x1, y1, theta1, xv[i], yv[i], length1, width1)) {
			/*std::cout << "For veh at pos: ("<<x2 << "," << y2 << ")\n";
			std::cout << "Four x points:" << xv[0] << "," << xv[1] << "," << xv[2] << "," << xv[3] << "\n";
			std::cout << "Four y points:" << yv[0] << "," << yv[1] << "," << yv[2] << "," << yv[3] << "\n";
			*/
			return 1;
		}
	}

	vertices(x1, y1, theta1, length1, width1, xv, yv);
	// examine whether a vertex of vehicle 1 lies within the rectangular of vehicle 2
	for (i = 0; i < 4; i++) {
		if (if_inside(x2, y2, theta2, xv[i], yv[i], length2, width2)) {
			/*std::cout << "For veh at pos: (" << x1 << "," << y1 << ")\n";
			std::cout << "Four x points:" << xv[0] << "," << xv[1] << "," << xv[2] << "," << xv[3] << "\n";
			std::cout << "Four y points:" << yv[0] << "," << yv[1] << "," << yv[2] << "," << yv[3] << "\n";
			*/
			return 1;
		}
	}

	return 0;
}

void 
LaneFreeSimulationPlugin::get_all_neighbors_internal(MSLaneFreeVehicle* lfveh, const MSEdge* current_edge, SortedVehiclesVector* current_edge_sorted_vehs, size_t veh_index, double distance, bool front, int cross_edge, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance) {

	

	// Get route object of vehicle
	const MSRoute veh_route = lfveh->get_vehicle()->getRoute();

	// This returns the set of edges corresponding to the vehicle's path, including the internal ones (on junctions)
	const ConstMSEdgeVector veh_edges = veh_route.getEdgeswInternal();

	// local position of vehicle
	double x_vid = lfveh->get_position_x();
	double remaining_distance = distance;

	// current edge id
	NumericalID edge_id = current_edge->getNumericalID();//lfveh->get_vehicle()->getLane()->getEdge().getNumericalID();//this will contain the edge, also accounting for intersection

	// find the current edge's index on the route array
	int my_edge_index{ veh_route.edge_index(current_edge) };

	// check whether edge was found
	if (my_edge_index == -1) {
		// last edge object pointer
		const MSEdge* last_edge = veh_edges.at(veh_edges.size() - 1);
		//this is an error only if the vehicle is not in the last edge already
		if (last_edge->getNumericalID() != edge_id) {
			std::cout << "Edge " << lfveh->get_vehicle()->getLane()->getEdge().getID() << " not found in route for vehicle " << lfveh->get_vehicle()->getID() << "!\n";

		}

		// if it is, then it means that the vehicle just exited
		// TODO check this again!		
		return;

	}

	

	// all vehicles in current edge, sorted according to (local) longitudinal position
	SortedVehiclesVector* sorted_vehs{ current_edge_sorted_vehs };//LaneFreeSimulationPlugin::getInstance()->get_sorted_vehicles_in_edge(edge_id);

	size_t size_edge = sorted_vehs->size();
	if (size_edge == 0) {
		std::cout << "ERROR: Sorted edge of vehicle found empty!\n"; // current edge should have at least ego veh_id		
		return;
	}

	size_t found{ veh_index };//binary_search_find_index(sorted_vehs, 0, (int)(size_edge - 1), lfveh->get_vehicle()->getNumericalID(), x_vid);

	if (found == -1) {

		std::cout << "\nVehicle " << lfveh->get_vehicle()->getID() << " not found in sorted vector of edge " << veh_edges[my_edge_index]->getID() << "!\n";
		return;
	}

	// starting from the next (or previous for upstream visibility) vehicle in the current segment 

	int front_back{ front ? +1 : -1 };
	size_t vehicle_index = found + front_back;
	//std::vector<NumericalID> neighbors;

	double my_pos{ x_vid }, neighbor_pos, neighbor_distance;
	NumericalID neighbor_id;

	// points to the temp neighbor
	const SUMOVehicle* neighbor_veh;

	double edge_length{ lf_plugin_get_edge_length(edge_id) };

	std::vector<MSLane*> internal_lanes;

	double global_pos_x{ 0 }, global_pos_y{ 0 }, global_theta{ 0 }, cos_theta{ 0 }, sin_theta{ 0 };
	// fill this information only for vehicles that use global coordinates
	if (lfveh->get_vehicle()->getGlobalCoordinatesControl()) {
		Position pos{ lfveh->get_vehicle()->getCachedGlobalPos() };
		double veh_length{lfveh->get_vehicle()->getLength()};
		global_theta - lfveh->get_vehicle()->getAngleRelative();
		cos_theta = cos(global_theta);
		sin_theta = sin(global_theta);
		global_pos_x = pos.x() - (veh_length)*cos_theta;
		global_pos_y = pos.y() - (veh_length)*sin_theta;
	}

	// do a while iterator, and break when we reach an edge in the path and a vehicle where lognitudinal distance is more than the specified one
	while (true) {

		// when we reach the last vehicle on the current edge we are searching
		if (vehicle_index >= size_edge || vehicle_index < 0) {
			// reset manually the longitudinal position for the next edge
			my_pos = my_pos - front_back * edge_length;
			
			if (lfveh->get_vehicle()->getGlobalCoordinatesControl()) {
				// check whether there are other internal lanes in the current edge based on the observed vehicles
				internal_lanes = veh_edges[my_edge_index]->getFromJunction()->getInternalLanes();

				if (internal_lanes.size() > 1) { // there are other augmented edges with different directions in the same junction that contain vehicles
					get_vehicles_from_other_direction_edges(lfveh->get_vehicle()->getNumericalID(), global_pos_x, global_pos_y, global_theta, front, internal_lanes, edge_id, neighbors_with_distance);
				}
				
			}
			// increase or decrease edge index
			if (front) {
				my_edge_index++;
			}
			else {
				my_edge_index--;
			}
			
			if (!cross_edge || my_pos < -(front_back)*distance || my_edge_index >= veh_edges.size() || my_edge_index < 0) {
				// terminating condition
				break;
			}
			
			
			if (veh_edges[my_edge_index] == nullptr) {
				size_edge = 0;
				edge_length = 0;

				continue;
			}
			edge_id = veh_edges[my_edge_index]->getNumericalID();

			edge_length = lf_plugin_get_edge_length(edge_id);

			sorted_vehs = LaneFreeSimulationPlugin::getInstance()->get_sorted_vehicles_in_edge(edge_id);

			if (sorted_vehs == nullptr) {//(sorted_vehs==nullptr) edge_id may not be initialized if no vehicles have entered
				size_edge = 0;
				vehicle_index = 0;
				continue;
			}
			vehicle_index = front ? 0 : (sorted_vehs->size() - 1);
			if ((size_edge = sorted_vehs->size()) == 0) {
				my_pos = my_pos - front_back * lf_plugin_get_edge_length(edge_id);
				continue;
			}
			
			// if we reach here, at least one vehicle should be located in edge edge_id


		}

		neighbor_veh = sorted_vehs->at(vehicle_index);
		neighbor_id = neighbor_veh->getNumericalID();

		if (lfveh->get_vehicle()->getGlobalCoordinatesControl()) {
			// use global coordinates to determine the distance, and check other edges

			transform_neighbor_vehicle_distance_and_add_to_neighbors((MSVehicle*)neighbor_veh, global_pos_x, global_pos_y, cos_theta, sin_theta, front, neighbors_with_distance);
		}
		else {
		
			// local position
			neighbor_pos = get_position_x(neighbor_id);

			// my_pos is updated whenever we change edge, so that it is w.r.t. the local coordinates of current edge (i.e., negative value if vehicle before the segments start)
			neighbor_distance = neighbor_pos - my_pos;

			if ((front && (neighbor_distance <= distance)) || ((!front) && (neighbor_distance > -distance))) {
				neighbors_with_distance.push_back(std::make_pair(neighbor_distance, (MSVehicle*)neighbor_veh));
				//std::cout << " " << neighbor_veh->getID() << " dist:" << neighbor_distance << "lat dist:" << get_relative_distance_y(veh_id, neighbor_id);
			}
			else {
				// In the standard case, we have sorted vehicles, therefore when a vehicle more than the specified distance, we can immediately break
				// the same principle does not apply in the global coordinates, because we cannot guarantee a complete ordering (i.e., the unfolded straight line)
				// As such, in global coordinates, we break according to the terminating condition that is located above in the while loop (when we reach an edge with a distance exceeding the specified range)
				break;		
			}
		}
		
		

		if (front) {
			vehicle_index++;
		}
		else {
			vehicle_index--;
		}
		

	}
}

void
LaneFreeSimulationPlugin::transform_neighbor_vehicle_distance_and_add_to_neighbors(MSVehicle* veh_ptr, double global_pos_x, double global_pos_y, double cos_theta, double sin_theta, bool front, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance) {
	
	

	// do a check first in for front vehicles. (also, consider the sorted_vector, in order to add them in order)
	Position pos_neigh{ veh_ptr->getCachedGlobalPos() };
	double length_neigh{ veh_ptr->getLength() };
	double theta_neigh{ veh_ptr->getAngleRelative() };

	double global_dx_neigh{ pos_neigh.x() - (length_neigh)*cos(theta_neigh) - global_pos_x };
	double global_dy_neigh{ pos_neigh.y() - (length_neigh)*sin(theta_neigh) - global_pos_y };
	
	// use Mehdi's method from sent document
	double dx_neigh_transform{ global_dx_neigh * cos_theta + global_dy_neigh * sin_theta };	
	if (front) {
		// vehicle is in front if dx>0. In case they are side by side (dx==0), we consider it in front when it is further on the left (dy>0)
		if (dx_neigh_transform > 0 || (dx_neigh_transform == 0 && (global_dy_neigh * cos_theta - global_dx_neigh * sin_theta) >= 0)) {
			neighbors_with_distance.push_back(std::make_pair(dx_neigh_transform, veh_ptr));
		}
	}
	else {
		// same idea for backwards observations
		if (dx_neigh_transform < 0 || (dx_neigh_transform == 0 && (global_dy_neigh * cos_theta - global_dx_neigh * sin_theta) < 0)) {
			neighbors_with_distance.push_back(std::make_pair(dx_neigh_transform, veh_ptr));
		}
	}
}

void
LaneFreeSimulationPlugin::get_vehicles_from_other_direction_edges(NumericalID veh_id, double global_pos_x, double global_pos_y, double global_theta, bool front, const std::vector<MSLane*>& internal_lanes, NumericalID current_edge_id, std::vector<std::pair<double, MSVehicle*>>& neighbors_with_distance) {



	MSEdge* edge_ptr;
	NumericalID edge_id;
	std::vector<const SUMOVehicle*>* vehs_in_edge{ nullptr };
	
	MSVehicle* veh_ptr;

	double sin_theta{ sin(global_theta) }, cos_theta{ cos(global_theta) };
	

	for (MSLane* lane_ptr : internal_lanes) {
		edge_ptr = &(lane_ptr->getEdge());

		edge_id = edge_ptr->getNumericalID();
		
		// skip the existing edge of our vehicle ved_id, as this is handled normally
		if (edge_id == current_edge_id) {
			continue;
		}

		vehs_in_edge = get_sorted_vehicles_in_edge(edge_id);
		
		if (vehs_in_edge == nullptr || (vehs_in_edge->size())==0) {
			continue;
		}
		// first, simply add all vehicles. Then, we can check through the angle the front/back
		
		for (const SUMOVehicle* sumo_veh_ptr : *vehs_in_edge) {

			// needs a casting, it is the same object
			veh_ptr = (MSVehicle*)sumo_veh_ptr;
			if (veh_ptr->getNumericalID() == veh_id) {
				//std::cout << "wtf, vehicle " << veh_ptr->getID() << " in edge: " << lf_plugin_get_edge_name(current_edge_id) << " is also in edge: " << edge_ptr->getID() << "\n";
				continue;
			}
			// call function with Mehdi's transformation
			transform_neighbor_vehicle_distance_and_add_to_neighbors(veh_ptr, global_pos_x, global_pos_y, cos_theta, sin_theta, front, neighbors_with_distance);
		}

		// sort the neighbors vector according to longutidinal distance
		//std::sort(neighbors_with_distance.begin(), neighbors_with_distance.end());
	}
}


void
LaneFreeSimulationPlugin::lf_simulation_checkCollisions(){
	//TODO approximate generalization according to bicycle model. while this is a generalization, i.e., it includes the standard case, since it requires more computational effort, we will have them as separate cases when estimating length and width for any given vehicle!
	
	MSEdgeVector edges_v = MSNet::getInstance()->getEdgeControl().getEdges();
	// NumericalID edges_size = edges_v.size();
	
	int i,j, n_v;
	std::vector<const SUMOVehicle*>* vehs_in_edge{nullptr};
	NumericalID edge_id;
	SortedVehiclesVectorEdges::iterator it_sorted_edge;
	MSVehicle* veh1;
	MSVehicle* veh2;
	MSLaneFreeVehicle* lfv1;
	MSLaneFreeVehicle* lfv2;
	double xv1, yv1, lv1, wv1, theta1=0, xv2, yv2, lv2, wv2, theta2=0, half_vwidth, dx, dy,roadwidth, roadlength;
	double xv1_gl, yv1_gl, theta1_gl;
	double distance_no_collide;
	for (MSEdge* edge : edges_v) {
		edge_id = edge->getNumericalID();

		//Following lines are replaced with function get_sorted_vehicles_in_edge
		//it_sorted_edge = sortedVehiclesVectorEdges.find(edge_id);
		//if (it_sorted_edge == sortedVehiclesVectorEdges.end()) {
		//	std::cout << "Sorted edge " << edge_id << " not found!\n";
		//}
		//vehs_in_edge = it_sorted_edge->second;
		
		vehs_in_edge = get_sorted_vehicles_in_edge(edge_id);
		if (vehs_in_edge == nullptr) {
			continue;
		}
		//*vehs_in_edge = edge->getVehicles(); //This can now be removed
		roadwidth = edge->getWidth();
		roadlength = edge->getLength();
		std::sort(vehs_in_edge->begin(), vehs_in_edge->end(), less_than_key()); //sorting is essentially performed here only
		n_v = vehs_in_edge->size();
		double veh_diag{ 0 };
		double front_distance;
		std::vector<std::pair<double, MSVehicle*>> neighbors_with_distance;
		for(i=0;i<n_v;i++){ //one potential improvement here is to have a memory for all the info requested, because for each vehicle, same info is requested multiple times
			neighbors_with_distance.clear();
			veh1 = (MSVehicle*)(*vehs_in_edge)[i];
			lv1 = veh1->getVehicleType().getLength();
			wv1 = veh1->getVehicleType().getWidth();
			lfv1 = find_vehicle_in_edge(veh1->getNumericalID(), edge_id); //we could somehow remove the need for this, maybe have the get_position_x, get_position_y as a function that gets the veh object as attribute
			xv1 = lfv1->get_position_x();
			yv1 = lfv1->get_position_y();
			// std::cout << "veh:" << veh1->getID() << " with length:" << lv1 << " and width:" << wv1 <<"\n";
			

			if (veh1->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE) {
				// this vehicle uses the bicycle model, meaning that we should consider the fact that it may have a non-zero orientation
				theta1 = veh1->getAngleRelativeAlways();
				// Code below is now deprecated, will be removed completely in future versions
				//update_vehicle_dimensions(&lv1, &wv1, abs(veh1->getAngleRelativeAlways()));		
				veh_diag = sqrt(pow(lv1, 2) + pow(wv1, 2));

				if (veh1->getGlobalCoordinatesControl()) {
					theta1_gl = veh1->getAngleRelative();
					const Position veh1glpos = veh1->getCachedGlobalPos();
					xv1_gl = veh1glpos.x() - (lv1)*cos(theta1_gl);
					yv1_gl = veh1glpos.y() - (lv1)*sin(theta1_gl);
				}
				front_distance = (veh_diag + max_vehicle_diag) / 2.0;
			}
			else {
				front_distance = (lv1 + max_vehicle_length) / 2.0;
			}
			// 
			// 
			// std::cout<< "veh:" << veh1->getID() << " updated length:" << lv1<<" and width:" << wv1<< " at angle:"<< veh1->getAngleRelative() << "\n";
			//std::cout << "veh:" << veh1->getID() << " at posx:" << xv1;
			//std::cout << "veh:" << veh1->getID() << " angle:" << veh1->computeAngle() * (180.0 / 3.141592653589793238463)<<"\n";
			
			half_vwidth = wv1 / 2;
			if (yv1 > (roadwidth - half_vwidth) || yv1 < half_vwidth) {
				event_vehicle_out_of_bounds(veh1->getNumericalID());
			}
			
			get_all_neighbors_internal(lfv1, edge, vehs_in_edge, (size_t)i, front_distance, true, 1, neighbors_with_distance);
			for(j = 0;j < neighbors_with_distance.size(); j++){				
				veh2 = neighbors_with_distance.at(j).second;
				lv2 = veh2->getVehicleType().getLength();
				wv2 = veh2->getVehicleType().getWidth();
				
				// Code below is now deprecated, will be removed completely in future versions
				if (veh2->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE) {
					// this vehicle also uses the bicycle model, meaning that we should consider the fact that it may have a non-zero orientation
					theta2 = veh2->getAngleRelativeAlways();
					//update_vehicle_dimensions(&lv2, &wv2, abs(veh2->getAngleRelativeAlways())); //(deprecated part) we could maybe save updated length and width, since for most vehicles we will do this computation more than once
				}
				lfv2 = find_vehicle_in_edge(veh2->getNumericalID(), veh2->getLane()->getEdge().getNumericalID());
				xv2 = lfv2->get_position_x();
				yv2 = lfv2->get_position_y();

				
				
				if (veh1->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE || veh2->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE) {
					// if vehicles utilize the global coordinate control, we need to use the global coordinates
					int collision_true{ false };
					if (veh1->getGlobalCoordinatesControl() && veh2->getGlobalCoordinatesControl()) {
						
						
						theta2 = veh2->getAngleRelative();
						const Position veh2glpos = veh2->getCachedGlobalPos();
						xv2 = veh2glpos.x() - (lv2) * cos(theta2);
						yv2 = veh2glpos.y() - (lv2) * sin(theta2);
						
						
						collision_true = collision_check_with_orientation(xv1_gl, yv1_gl, theta1_gl, lv1, wv1, xv2, yv2, theta2, lv2, wv2);
						
					}
					else {
						collision_true = collision_check_with_orientation(xv1, yv1, theta1, lv1, wv1, xv2, yv2, theta2, lv2, wv2);
					}

					if (collision_true) {
						event_vehicles_collide(veh1->getNumericalID(), veh2->getNumericalID());
						//std::cout << veh1->getID() << " at pos (" << xv1 << "," << yv1 << ") with angle " << theta1 << "and dimensions ("<< lv1<<","<<wv1 << ") ;" << veh2->getID() << " at pos (" << xv2 << "," << yv2 << ") with angle " << theta2 << "and dimensions (" << lv2 << "," << wv2 << ") \n";
						MSNet::getInstance()->getVehicleControl().registerCollision();
						lfv1->just_collided_with(veh2->getNumericalID());
						if (!(lfv1->has_collided_with(veh2->getNumericalID()) || lfv2->has_collided_with(veh1->getNumericalID()))) {
							MSNet::getInstance()->getVehicleControl().registerCollisionNoConsecutives();
							//system("pause");
						}

					}
					
					
					//skip the standard condition for collision check
					continue;
				}

				// calculate dx with local coordinates, in order to check
				dx = abs(xv2 - xv1);
				dy = abs(yv2 - yv1);
				

				if((dx<((lv1 + lv2)/2)) && (dy<((wv1 + wv2)/2))){
					event_vehicles_collide(veh1->getNumericalID(), veh2->getNumericalID());
					MSNet::getInstance()->getVehicleControl().registerCollision();

					lfv1->just_collided_with(veh2->getNumericalID());
					if (!(lfv1->has_collided_with(veh2->getNumericalID()) || lfv2->has_collided_with(veh1->getNumericalID()))) {
						MSNet::getInstance()->getVehicleControl().registerCollisionNoConsecutives();
						//system("pause");
					}
				}
				theta2 = 0;
				if(dx>(lv1+max_vehicle_length)/2){ // since vehicles are sorted based on x pos, we stop searching downstream vehicles for collisions when the dx distance exceeds the one corresponding to a vehicle with the maximum length within the network TODO: orientiation should be taken into account for the bicycle model
					break;
				}
			}
			lfv1->empty_collided_set();

			theta1 = 0;

			
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

	std::vector<const SUMOVehicle*>* svehs_edge;
	for (SortedVehiclesVectorEdges::iterator it = sortedVehiclesVectorEdges.begin(); it != sortedVehiclesVectorEdges.end(); it++) {
		svehs_edge = it->second;
		
		delete svehs_edge;

	}
	sortedVehiclesVectorEdges.clear();
}

void
LaneFreeSimulationPlugin::insert_vehicle(MSVehicle* veh){
	NumericalID edge_id = veh->getLane()->getEdge().getNumericalID();
	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(edge_id);
	SortedVehiclesVector* sortedvehicles;
	if(it==allVehiclesMapEdges.end()){
		VehicleMap* vm = new VehicleMap;
		allVehiclesMapEdges.insert(std::make_pair(edge_id,vm));

		sortedvehicles = new SortedVehiclesVector;
		sortedVehiclesVectorEdges.insert(std::make_pair(edge_id, sortedvehicles));
		it = allVehiclesMapEdges.find(edge_id);
	}
	else {

		SortedVehiclesVectorEdges::iterator it_q = sortedVehiclesVectorEdges.find(edge_id);
		sortedvehicles = it_q->second;
	}
	
	VehicleMap* vm = it->second;
	
	

	MSLaneFreeVehicle* new_veh = new MSLaneFreeVehicle(veh);
	sortedvehicles->push_back((SUMOVehicle*)veh);
	NumericalID veh_nid = veh->getNumericalID();
	vm->insert(std::make_pair(veh_nid,new_veh));
	double lv = veh->getVehicleType().getLength();
	bool updated_max_dim{ false };
	if(lv > max_vehicle_length){
		max_vehicle_length = lv;
		updated_max_dim = true;
	}
	double wv = veh->getVehicleType().getWidth();
	if (wv > max_vehicle_width) {
		max_vehicle_width = wv;
		updated_max_dim = true;
	}

	if (updated_max_dim) {
		max_vehicle_diag = sqrt(pow(max_vehicle_length, 2) + pow(max_vehicle_width, 2));
	}

	InsertedAdditionalInitStatus::iterator it_l = insertedAdditionalInitStatus.find(veh_nid);
	if(it_l!=insertedAdditionalInitStatus.end()){
		double pos_y = (it_l->second)[0];
		double speed_y = (it_l->second)[1];
		double theta = (it_l->second)[2];

		bool use_global_coordinates = (it_l->second)[4];
		if (!use_global_coordinates) {
			new_veh->set_position_y(pos_y);
		}
		else {
			double pos_x = (it_l->second)[3];
			Position pos_global_init(pos_x + (lv / 2) * cos(theta), pos_y + (lv / 2) * sin(theta));
			//std::cout << "Initial global position for veh ("<< veh->getID() <<"):" << pos_x + (lv / 2) * cos(theta) << "," << pos_y + (lv / 2) * sin(theta)<< "\n";
			double x_local=0, y_local=0;
			const MSLane* init_lane = veh->getLane();
			convert_to_local_coordinates(&x_local, &y_local, pos_global_init, init_lane);
			new_veh->set_position_x_front(x_local);
			new_veh->set_position_y_front(y_local);
			//std::cout << "Initial local position for veh (" << veh->getID() << "):" << x_local << "," << y_local << "\n";
			veh->setGlobalCoordinatesControl(true);
			veh->setCachedGlobalPos(pos_global_init.x(), pos_global_init.y());
			
		}
		//std::cout << "theta:" << theta << "\n";
		
		new_veh->set_speed_y(speed_y);

		// only vehicles adhering to the bicycle model can have an initial non-zero orientation
		if (veh->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE) {
			//std::cout << "Angle for veh " << veh->getID() << " " << theta << "\n";
			new_veh->set_angle_relative(theta);
		}
		else if (theta != 0) {
			std::cout << "Warning! Non-zero initial orientation selected for vehicle:" << new_veh->get_vehicle()->getID() << " will be omitted, since it does not adhere to the bicycle model!\n";
		}
		insertedAdditionalInitStatus.erase(veh_nid);
	}	
	// double init_pos_x=new_veh->get_position_x(), init_pos_y=new_veh->get_position_y(), init_speed_x=new_veh->get_speed_x();
	
	event_vehicle_enter(veh_nid);

	// first condition means that vehicle is not inserted through the API
	if (it_l == insertedAdditionalInitStatus.end() && veh->getVehicleType().getParameter().cmdModel == SUMO_TAG_LF_CMD_BICYCLE && veh->getGlobalCoordinatesControl()) {
		// in case a new vehicle from any demand adopts the bicycle model, and used global coordinates, initial angle should not be zero, but rather the one parallel to the residing road, so 
		const MSLane* init_lane = veh->getLane();
		double init_theta = init_lane->getShape().rotationAtOffset(init_lane->interpolateLanePosToGeometryPos(veh->getPositionOnLane()));
		new_veh->set_angle_relative(init_theta);
		//std::cout << "Veh passed from here with init theta:"<< init_theta<<"\n";
		
	}

	// new_veh->set_position_x(init_pos_x);
	// new_veh->set_position_y(init_pos_y);
	// new_veh->set_speed_x(init_speed_x);
	//std::cout << "Vehicle "<< veh->getNumericalID() <<" inserted!\n";
	

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
		std::cout << "Vehicle "<< veh_id <<" not found in edge "<< edge_id <<"!\n";
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

void
LaneFreeSimulationPlugin::change_edge(MSVehicle* veh){
	//std::cout << "Start with change "<< veh->getID()<<" !\n";
	NumericalID new_edge_id = veh->getLane()->getEdge().getNumericalID();
	NumericalID old_edge_id = find_stored_edge(veh);
	//std::cout << "old edge " << old_edge_id << " new edge " << new_edge_id << " with name:"<< veh->getLane()->getEdge().getID() <<"\n";
	if(old_edge_id==new_edge_id){
		return;
	}

	VehicleMap* old_edge_vm = allVehiclesMapEdges[old_edge_id];
	
	NumericalID veh_id = veh->getNumericalID();
		

	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(new_edge_id);
	
	if(it==allVehiclesMapEdges.end()){		
		VehicleMap* new_edge = new VehicleMap;
		allVehiclesMapEdges.insert(std::make_pair(new_edge_id, new_edge));
		std::vector<const SUMOVehicle*>* sortedvehicles = new std::vector<const SUMOVehicle*>;
		sortedVehiclesVectorEdges.insert(std::make_pair(new_edge_id, sortedvehicles));
	}

	VehicleMap* new_edge_vm = allVehiclesMapEdges[new_edge_id];
	MSLaneFreeVehicle* ch_veh = (*old_edge_vm)[veh->getNumericalID()];
	new_edge_vm->insert(std::make_pair(veh->getNumericalID(),ch_veh));

	old_edge_vm->erase(veh->getNumericalID());

	const SUMOVehicle* change_sumo_vehicle = (const SUMOVehicle*)veh;
	SortedVehiclesVector* old_edge_vehs = sortedVehiclesVectorEdges[old_edge_id];
	const SUMOVehicle* iter_vehicle;

	//iterate from the end to the beginning, to find the vehicle (should be towards the end of the edge)
	size_t i = old_edge_vehs->size()-1;
	size_t found_index = -1;	
	//std::cout << "Start with old removal (size "<<i+1<<") !\n";
	for (SortedVehiclesVector::reverse_iterator  it_sv = old_edge_vehs->rbegin(); it_sv != old_edge_vehs->rend(); it_sv++,i--) {
		iter_vehicle = *it_sv;	
		
		if (iter_vehicle->getNumericalID()==veh_id) {
			found_index = i;
			
			old_edge_vehs->erase(old_edge_vehs->begin()+i);
			break;
		}
		//std::cout << "iter!\n";
	}
	//std::cout << "End with old removal, (new size "<< old_edge_vehs->size()<<") !\n";
	
	SortedVehiclesVector* new_edge_vehs = sortedVehiclesVectorEdges[new_edge_id];
	
	//std::cout << "hello with size"<< new_edge_vehs->size()<<"\n";
	
	new_edge_vehs->insert(new_edge_vehs->begin(),change_sumo_vehicle);

	//std::cout << "End with change with size" << new_edge_vehs->size() << "\n"; 
	
	// std::cout << "Vehicle "<< veh->getNumericalID() <<" changed from edge " << old_edge_id <<" to edge " << new_edge_id<<" !\n";

	
}

void 
LaneFreeSimulationPlugin::remove_vehicle(MSVehicle* veh){

	const MSLane* vehlane = veh->getLane();
	// if the following condition is met, then exit function is called for a vehicle that has never beed inserted (e.g., it stayed in the virtual queue)
	if (vehlane == nullptr) {
		return;
	}
	NumericalID edge_id = vehlane->getEdge().getNumericalID();
	VehicleMapEdges::iterator it = allVehiclesMapEdges.find(edge_id);
	
	if(it==allVehiclesMapEdges.end()){		
		std::cout << "Edge " << edge_id << " not found!\n";
		return;
	}
	NumericalID veh_id = veh->getNumericalID();	
	VehicleMap* vm = it->second;

	// if the following condition is met, then exit function is called for a vehicle that has never beed inserted (e.g., it stayed in the virtual queue)
	if (vm->find(veh_id) == vm->end()) {
		return;
	}
	//std::cout << "Veh " << veh->getID()<< " has arrived:" << veh->hasArrived() << "\n";
	
	event_vehicle_exit(veh_id, (int) veh->hasArrived());
	MSLaneFreeVehicle* ch_veh = (*vm)[veh_id];
	delete ch_veh;
	vm->erase(veh_id);
	
	SortedVehiclesVectorEdges::iterator it_q = sortedVehiclesVectorEdges.find(edge_id);
	if (it_q == sortedVehiclesVectorEdges.end()) {
		std::cout << "Edge " << edge_id << " not found in sortedvehicles memory!\n";
		return;
	}
	SortedVehiclesVector* sortedvehicles = it_q->second;
	const SUMOVehicle* iter_vehicle;
	int i = sortedvehicles->size() - 1;
	for (SortedVehiclesVector::reverse_iterator it_rv = sortedvehicles->rbegin(); it_rv != sortedvehicles->rend(); it_rv++,i--) {
		iter_vehicle = *it_rv;

		if (iter_vehicle->getNumericalID() == veh_id) {
			
			sortedvehicles->erase(sortedvehicles->begin() + i);
			break;
		}
		
	}

	// std::cout << "Vehicle "<< veh->getID() <<" exited the network!\n";
}

LaneFreeSimulationPlugin*
LaneFreeSimulationPlugin::getInstance(void) {
    if (myInstance != nullptr) {
        return myInstance;
    }
    throw ProcessError("The laneFreePlugin instance was not yet constructed.");
}


SortedVehiclesVector*
LaneFreeSimulationPlugin::get_sorted_vehicles_in_edge(NumericalID edge_id) {
	SortedVehiclesVectorEdges::iterator it_sorted_edge = sortedVehiclesVectorEdges.find(edge_id);
	if (it_sorted_edge != sortedVehiclesVectorEdges.end()) {
		return it_sorted_edge->second;
	}
	//std::cout << "Sorted edge " << edge_id << " not found!\n";
	return nullptr;
}


// properly adapt existing codebase here, in order to convert global coordinates to local
void 
LaneFreeSimulationPlugin::convert_to_local_coordinates(double* x_pos_local, double* y_pos_local, Position& pos, const MSLane* mylane) {


	*x_pos_local = mylane->interpolateGeometryPosToLanePos(mylane->getShape().nearest_offset_to_point25D(pos, false));

	double angle_lane = mylane->getShape().beginEndAngle();
	const double perpDist = mylane->getShape().distance2D(pos, false);
	*y_pos_local = perpDist;// std::min(perpDist, 0.5 * (mylane->getWidth() + myveh->getVehicleType().getWidth() - MSGlobals::gLateralResolution));
	PositionVector tmp = mylane->getShape();
	tmp.move2side(-*y_pos_local); // moved to left
	if (tmp.distance2D(pos) > perpDist) {
		*y_pos_local = -*y_pos_local;
	}

}

//to be removed
//double MSLaneFreeVehicle::last_init_pos_y=0;
//double MSLaneFreeVehicle::last_v_width=0;

LastVehicleStatus MSLaneFreeVehicle::last_veh_status = LastVehicleStatus();
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
    
	/*
	double vNext = MSCFModel::finalizeSpeed(veh, vPos);
    if (myAdaptationFactor != 1.) {
        VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
        vars->levelOfService += (vNext / veh->getLane()->getVehicleMaxSpeed(veh) - vars->levelOfService) / myAdaptationTime * TS;
    }
	*/

    // if(vNext>25){
    // 	vNext = 25;
    // }
    // vNext = 25;
    // double v_lf_model = LaneFreeSimulationPlugin::getInstance()->get_veh_speed(veh->getNumericalID());
    // MSLaneFreeVehicle *lfveh =  LaneFreeSimulationPlugin::getInstance()->find_vehicle_in_edge(veh->getLane()->getEdge().getNumericalID(),veh->getNumericalID());


	double vNext(0);
    MSLaneFreeVehicle *lfveh =  LaneFreeSimulationPlugin::getInstance()->find_vehicle(veh->getNumericalID());
    if(lfveh==nullptr){
    	std::cout<<"Issue\n";
    	return vNext;
    }
    
    
    vNext = lfveh->apply_acceleration_internal();
    
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
