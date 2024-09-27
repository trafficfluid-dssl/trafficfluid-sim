/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2020 German Aerospace Center (DLR) and others.
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
/// @file    MSRoute.cpp
/// @author  Daniel Krajzewicz
/// @author  Friedemann Wesner
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Sept 2002
///
// A vehicle route
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <algorithm>
#include <limits>
#include <utils/common/FileHelpers.h>
#include <utils/common/RGBColor.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSEdge.h"
#include "MSLane.h"
#include "MSRoute.h"

// LFPlugin Begin
#include <utils/shapes/ShapeContainer.h>
#include <microsim/cfmodels/MSCFModel_LaneFree.h>
// LFPlugin End


// ===========================================================================
// static member variables
// ===========================================================================
MSRoute::RouteDict MSRoute::myDict;
MSRoute::RouteDistDict MSRoute::myDistDict;
#ifdef HAVE_FOX
FXMutex MSRoute::myDictMutex(true);
#endif


// ===========================================================================
// member method definitions
// ===========================================================================
MSRoute::MSRoute(const std::string& id,
                 const ConstMSEdgeVector& edges,
                 const bool isPermanent, const RGBColor* const c,
                 const std::vector<SUMOVehicleParameter::Stop>& stops) :
    Named(id), myEdges(edges), myAmPermanent(isPermanent),
    myReferenceCounter(isPermanent ? 1 : 0),
    myColor(c),
    myPeriod(0),
    myCosts(-1),
    mySavings(0),
    myReroute(false),
    myStops(stops),
    visualizeLeftBoundary(false)
    // LFPlugin Begin
    ,
    hasLeftBoundary(false),
    hasRightBoundary(false),
    num_of_steps_visualization_update_delay(1),
    visualization_update_remaining_steps(0)
    // LFPlugin End
    {
    // LFPlugin Begin
    // initialize myEdgeswInternal
    const MSEdge* tmp_edge_prev = nullptr;
    const MSEdge* tmp_edge_internal;
    std::cout << "checking...: " << id << "\n";
    for (ConstMSEdgeVector::iterator it = myEdges.begin(); it != myEdges.end(); it++) {

        if (tmp_edge_prev != nullptr) {
            while ((tmp_edge_internal = tmp_edge_prev->getInternalFollowingEdgeVehicleOnly((*it))) != nullptr) { // we need a while loop in case of an intersection (where we have two consecutive internal lanes)
                //std::cout << "\t\t: " << tmp_edge_internal->getID() << "\n";
                myEdgeswInternal.push_back(tmp_edge_internal);
                tmp_edge_prev = tmp_edge_internal;
            }
        }
        myEdgeswInternal.push_back((*it));



        tmp_edge_prev = (*it);
    }

    for (ConstMSEdgeVector::iterator it = myEdges.begin(); it != myEdges.end(); it++) { // now for bike lanes

        if (tmp_edge_prev != nullptr) {
            while ((tmp_edge_internal = tmp_edge_prev->getInternalFollowingEdgeBikeOnly((*it))) != nullptr) { // we need a while loop in case of an intersection (where we have two consecutive internal lanes)
                //std::cout << "\t\t: " << tmp_edge_internal->getID() << "\n";
                myEdgeswInternalBike.push_back(tmp_edge_internal);
                tmp_edge_prev = tmp_edge_internal;
            }
        }
        myEdgeswInternalBike.push_back((*it));



        tmp_edge_prev = (*it);
    }
    // LFPlugin End
    }

// LFPlugin Begin

void 
MSRoute::getBoundaryShape(std::vector<double>& boundaryLevelPoints, std::vector<double>& boundaryOffsets, std::vector<double>& boundarySlopes, double step, PositionVector& boundaryShape) {
    
    // we choose the path direction according to the first lane on the path
    const std::vector<MSLane*> myLanes = myEdges.at(0)->getLanes();
    if (myLanes.size() == 0) {
        std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty lane set for first edge!\n";
        return;
    }
    MSLane* myFirstLane{ myLanes.at(0) };


    // we consider routes with consistent angle
    double pathAngle = myFirstLane->getShape().angleAt2D(0);

    double sign_coeff = cos(pathAngle) > 0 ? +1. : -1.;


    // in global coordinates
    double x_origin = myFirstLane->getShape()[0].x();

    // find x position of the route's end in global coordinates. It will be the last point of the last lane's shape
    const std::vector<MSLane*> myLanes_last = myEdges.at(myEdges.size() - 1)->getLanes();
    MSLane* myLastLane{ myLanes_last.at(myLanes_last.size() - 1) };
    double x_end = myLastLane->getShape()[myLastLane->getShape().size() - 1].x();

    //std::cout << "For route:" << getID() << " x start:" << x_origin << " and x end:" << x_end << "\n";
    bool finish = false;

    // we want to calculate the y_pos of the boundary
    double x_pos_tmp=0,y_pos;
    
    for (double x_pos = x_origin; x_pos * sign_coeff <= x_end * sign_coeff; x_pos = x_pos + sign_coeff * step) {
        
        // y_pos gets the boundary's value
        LaneFreeSimulationPlugin::getInstance()->boundary_value(0.5, sign_coeff, boundaryLevelPoints, boundarySlopes, boundaryOffsets, x_pos, 0., &y_pos, NULL);

        boundaryShape.push_back(Position(x_pos, y_pos));
        x_pos_tmp = x_pos;
    }
    
    if (x_pos_tmp != x_end) {
        // y_pos gets the boundary's value
        LaneFreeSimulationPlugin::getInstance()->boundary_value(0.5, sign_coeff, boundaryLevelPoints, boundarySlopes, boundaryOffsets, x_end, 0., &y_pos, NULL);
        boundaryShape.push_back(Position(x_end, y_pos));

    }
    
}

void 
MSRoute::setLeftBoundary(std::string& leftBoundaryLevelPointsString, std::string& leftBoundarySlopesString, std::string& leftBoundaryOffsetsString, std::string& influencedBy, std::string& visualizeLeftBoundaryColor, std::string& vizualizeLeftBoundaryStep, std::string& vizualizeLeftBoundaryLineWidth, std::string& vizualizeLeftBoundaryUpdateDelaySeconds) {
    hasLeftBoundary = true;

    // use StringTokenizer to get the string xml value as a vector of string values
    StringTokenizer lbpoints(leftBoundaryLevelPointsString);
    std::vector<std::string> lbpointsStringVector = lbpoints.getVector();

    StringTokenizer lbslopes(leftBoundarySlopesString);
    std::vector<std::string> lbslopesStringVector = lbslopes.getVector();

    StringTokenizer lboffsets(leftBoundaryOffsetsString);
    std::vector<std::string> lboffsetsStringVector = lboffsets.getVector();

    if (lbslopesStringVector.size() != lboffsetsStringVector.size()) {// || (lbpointsStringVector.size() - 1) != lbslopesStringVector.size()) {
        std::cout << "Error when parsing Left Boundary for Route:" << getID() << ".";
        std::cout << "Size of leftBoundarySlopes does not match leftBoundaryOffsets.\n";
        hasLeftBoundary = false;
        return;
    }

    if ((lbpointsStringVector.size() - 1) != lbslopesStringVector.size()) {
        std::cout << "Error when parsing Left Boundary for Route:" << getID() << ".";
        std::cout << "Size of leftBoundaryLevelPoints does not match leftBoundaryOffsets.\n";
        hasLeftBoundary = false;
        return;
    }

    if (lbpointsStringVector.size() == 0 || lbslopesStringVector.size() == 0 || lboffsetsStringVector.size() == 0) {
        std::cout << "Error when parsing Left Boundary for Route:" << getID() << ".";
        std::cout << "At least one of the associated elements is empty.\n";
        hasLeftBoundary = false;
        return;
    }

    // convert the string elements to double
    double element_value;

    int points_size = lbpointsStringVector.size();
    for (int i = 0; i < points_size; i++) {

        element_value = StringUtils::toDouble(lbpointsStringVector.at(i));
        leftBoundaryLevelPoints.push_back(element_value);
        // initialize leftBoundaryLevelPointsEpsilonCoefficients with epsilon=1, i.e., the same values with leftBoundaryLevelPoints
        leftBoundaryLevelPointsEpsilonCoefficients.push_back(element_value);

        // visualized boundary also will start with the same value
        leftBoundaryLevelPointsEpsilonCoefficientsVisualized.push_back(element_value);
        leftBoundaryLevelPointsEpsilonCoefficientsStep.push_back(0.);


        if (i == points_size - 1) {
            continue; // or break; It is the last iteration so they are equivalent
        }

        element_value = StringUtils::toDouble(lbslopesStringVector.at(i));
        leftBoundarySlopes.push_back(element_value);

        element_value = StringUtils::toDouble(lboffsetsStringVector.at(i));
        leftBoundaryOffsets.push_back(element_value);

        
    }

    //std::cout << "For route:" << getID() << "\n";

    // obtain local position of the offsets. This is useful for calculating the densities of each segment formed by two offsets
    const ConstMSEdgeVector veh_edges = getEdgeswInternal();
    int i = -1;
    size_t offset_idx = 0;    
    for (const MSEdge* edge_ptr : veh_edges) {
        i++;
        if (edge_ptr->isInternal() && i < (veh_edges.size() - 1) && (edge_ptr->getLanes().at(edge_ptr->getLanes().size() - 1)->getShape()[0] == (veh_edges.at(i + 1))->getLanes().at(edge_ptr->getLanes().size() - 1)->getShape()[0])) {
            continue;
        }
        //std::cout << "Edge:" << edge_ptr->getID() << " " << edge_ptr->isInternal() << " with starting point:" << edge_ptr->getLanes().at(edge_ptr->getLanes().size() - 1)->getShape()[0] << " and length:" << edge_ptr->getLength() << " ";
        while(fabs(edge_ptr->getLanes().at(edge_ptr->getLanes().size() - 1)->getShape()[0].x() - leftBoundaryOffsets.at(offset_idx)) < edge_ptr->getLength()) {
            //std::cout << "\nOffset:" << leftBoundaryOffsets.at(offset_idx) << " is on edge:" << edge_ptr->getID() << " at local pos x:" << leftBoundaryOffsets.at(offset_idx) - edge_ptr->getLanes().at(edge_ptr->getLanes().size() - 1)->getShape()[0].x() << "\n";
            localEdgePosLeftBoundaryOffsets.push_back(std::make_pair(edge_ptr->getNumericalID(), fabs(leftBoundaryOffsets.at(offset_idx) - edge_ptr->getLanes().at(edge_ptr->getLanes().size() - 1)->getShape()[0].x())));
            offset_idx++;
            if (offset_idx == leftBoundaryOffsets.size()) {
                break;
            }
        }

        if (offset_idx == leftBoundaryOffsets.size()) {
            break;
        }
        
    }
    //std::cout << "\n";
    size_t influencedIdx = 0;
    MSRoute* influencer{ nullptr };
    if (influencedBy.size() > 0 && influencedBy != "") {
        //this variable is not empty

        influencer = (MSRoute*)MSRoute::dictionary(influencedBy);
        if (influencer == nullptr) {
            std::cout << "Error! When parsing information for route " << getID() << ", influencer route " << influencedBy << " could not be found!\n";
            return;
        }
        influencedIdx = influencer->addInfluencedRoute(this, leftBoundaryOffsets);
        
    }


    // visualization is not specified
    if (visualizeLeftBoundaryColor.empty() || visualizeLeftBoundaryColor == "") {
        //std::cout << "Returns here!\n";
        return;
    }
    //std::cout << "Called for route " << getID() << "\n";
    visualizeLeftBoundary = true;
    const std::vector<MSLane*> myLanes = myEdges.at(0)->getLanes();
    if (myLanes.size() == 0) {
        std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty lane set for first edge!\n";
        return;
    }
    MSLane* myFirstLane{ myLanes.at(0) };


    // in global coordinates
    double x_origin = myFirstLane->getShape()[0].x();

    // find x position of the route's end in global coordinates. It will be the last point of the last lane's shape
    const std::vector<MSLane*> myLanes_last = myEdges.at(myEdges.size() - 1)->getLanes();
    MSLane* myLastLane{ myLanes_last.at(myLanes_last.size() - 1) };
    double x_end = myLastLane->getShape()[myLastLane->getShape().size() - 1].x();

    // by default we select a step that will create visualization segments of at least twice the amount of offset points
    leftBoundaryVisualizationStep = abs(x_end - x_origin) / (8 * leftBoundaryOffsets.size());

    if ((!vizualizeLeftBoundaryStep.empty()) && vizualizeLeftBoundaryStep != "") {
        double step_parse = StringUtils::toDouble(vizualizeLeftBoundaryStep);

        if (step_parse > leftBoundaryVisualizationStep) {
            std::cout << "Warning! Step selected for the vizualisation of left boundary at route " << getID() << " will be overwritten! We want the step selection to provide at least twice the number of offset points!\n";
        }
        else {
            leftBoundaryVisualizationStep = step_parse;
        }

    }

    // default value
    double visualizeBoundaryLineWidth = 0.1;
    if ((!vizualizeLeftBoundaryLineWidth.empty()) && vizualizeLeftBoundaryLineWidth != "") {
        double line_width = StringUtils::toDouble(vizualizeLeftBoundaryLineWidth);
        if (line_width <= 0) {
            std::cout << "Line width for left boundary at route " << getID() << " cannot be non-positive! Default value of 0.1 will be selected instead.\n";
        }
        else {
            visualizeBoundaryLineWidth = line_width;
        }
    }


    




    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();
    PositionVector pShape;

    
 
    getBoundaryShape(leftBoundaryLevelPoints, leftBoundaryOffsets, leftBoundarySlopes, leftBoundaryVisualizationStep, pShape);
    
    RGBColor col = RGBColor::parseColor(visualizeLeftBoundaryColor);
    
    if (!shapeCont.addPolygon("leftBoundary_"+getID(), "leftBoundary", col, 130.-(double)influencedIdx, Shape::DEFAULT_ANGLE, Shape::DEFAULT_IMG_FILE, Shape::DEFAULT_RELATIVEPATH, pShape, false, false, visualizeBoundaryLineWidth)) {
        std::cout << "Error! could not visualize left boundary of route:"<< getID()<<"\n";
    }
    
    
    // we also need to update the veh out of bounds event function


    

    if (influencer != nullptr) {
        // if this is influeced by another route, this will inherit its delay
        num_of_steps_visualization_update_delay = influencer->getVisualizerUpdateDelaySteps();
    }
    else {
        // default value is 2 seconds
        double visualizeBoundariesUpdateDelaySeconds = 2.;
        if ((!vizualizeLeftBoundaryUpdateDelaySeconds.empty()) && vizualizeLeftBoundaryUpdateDelaySeconds != "") {
            double parse_delay = StringUtils::toDouble(vizualizeLeftBoundaryUpdateDelaySeconds);
            
            if (parse_delay < 0) {
                std::cout << "Delay for epsilon update for left boundary at route " << getID() << " cannot be negative! Default value of 2 seconds will be selected instead.\n";
            }
            else {
                visualizeBoundariesUpdateDelaySeconds = parse_delay;
            }
        }

        if (visualizeBoundariesUpdateDelaySeconds == 0.) {
            num_of_steps_visualization_update_delay = 1;
        }
        else {
            num_of_steps_visualization_update_delay = std::ceil(visualizeBoundariesUpdateDelaySeconds / TS);
        }
        //std::cout << "Selected step is:"<< num_of_steps_visualization_update_delay << " with delay:" << visualizeBoundariesUpdateDelaySeconds << " and step:" << TS << "\n";
    }

    

    // we use ceil in case the value of seconds is not proportional to the time-step selected
    
    
}

void
MSRoute::updateLeftBoundaryLevelPointsEpsilonCoefficients(std::vector<double>& leftBoundaryEpsilons){
    
    double boundaries_width;
    
    //std::cout << "Update route " << getID() << " with epsilons: ";
   /* for (double elem : leftBoundaryEpsilons) {
        std::cout << elem<<" ";
    }*/
    //std::cout << "\n";
    size_t boundaries_size = leftBoundaryLevelPoints.size();
    if (boundaries_size == 0) {
        std::cout << "Error for route " << getID() << "! Left boundary level points are not defined!\n";
    }

    if (leftBoundaryEpsilons.size() != boundaries_size) {
        std::cout << "Error for route " << getID() << "! Size of epsilon vector is:" << leftBoundaryEpsilons.size() << " whereas the defined size of left boundary level points is: " << boundaries_size << "\n";
    }

    
    double epsilon_val, deviation;
    for (int i = 0; i < leftBoundaryLevelPoints.size(); i++) {
        boundaries_width = leftBoundaryLevelPoints.at(i) - rightBoundaryConstantLevelPoint;
        
        epsilon_val = leftBoundaryEpsilons.at(i);
        if (epsilon_val < 0 || epsilon_val>2) {
            std::cout << "All epsilon values should lie within the range 0<epsilon<2! Error for route " << getID() << " at epsilon index " << i << ". This value will be neglected\n";
            epsilon_val = 1;
        }
        // update the left boundary
        leftBoundaryLevelPointsEpsilonCoefficients.at(i) = rightBoundaryConstantLevelPoint + boundaries_width * epsilon_val;

        // calculate the deviation of the visualized boundary and the updated value
        deviation = leftBoundaryLevelPointsEpsilonCoefficients.at(i) - leftBoundaryLevelPointsEpsilonCoefficientsVisualized.at(i);
        //std::cout << "deviation is:" << deviation << " where leftboundary is at:"<< leftBoundaryLevelPointsEpsilonCoefficients.at(i) << ", and visualized at:"<< leftBoundaryLevelPointsEpsilonCoefficientsVisualized.at(i) << "\n";
        // calculate the visualization step for every time-step
        leftBoundaryLevelPointsEpsilonCoefficientsStep.at(i) = deviation / (double)num_of_steps_visualization_update_delay;
        //std::cout << "norm deviation is:" << leftBoundaryLevelPointsEpsilonCoefficientsStep.at(i) << "\n";
    }


    // reset the visualization delay counter
    visualization_update_remaining_steps = num_of_steps_visualization_update_delay;
    //std::cout << "remaining steps:" << visualization_update_remaining_steps << "\n";

    std::vector<double> influenced_epsilons;
    size_t influenced_size_epsilons, start_idx, end_idx, start_my_idx, end_my_idx, my_idx;
    MSRoute* influenced;
    for (size_t influenced_idx = 0; influenced_idx < influencedRoutes.size(); influenced_idx++) {
        influenced_epsilons.clear();
        influenced = influencedRoutes.at(influenced_idx);
        influenced_size_epsilons = influenced->getLeftBoundaryLevelPoints().size();

        // initialize all epsilons with 1
        influenced_epsilons = std::vector<double>(influenced_size_epsilons, 1);
        
        start_idx = influencedRoutesEpsilonIndicesStartEnd.at(influenced_idx).first;
        end_idx = influencedRoutesEpsilonIndicesStartEnd.at(influenced_idx).second;
        
        start_my_idx = influencedRoutesInfluencerEpsilonIndicesStartEnd.at(influenced_idx).first;
        end_my_idx = influencedRoutesInfluencerEpsilonIndicesStartEnd.at(influenced_idx).second;
        my_idx = start_my_idx;
        for (size_t epsilon_idx = start_idx; epsilon_idx <= end_idx; epsilon_idx++) {
            if (my_idx == end_my_idx + 1) {
                std::cout << "Error in the association of epsilons for influencer route " << getID() << " and influenced " << influenced->getID() << "\n";
            }

            // override the associated epsilons from the influencer
            influenced_epsilons.at(epsilon_idx) = leftBoundaryEpsilons.at(my_idx);
            my_idx++;
            
        }
        /*std::cout << "Update influenced route " << influenced->getID() << " with epsilons:";
        for (double elem : influenced_epsilons) {
            std::cout << elem << " ";
        }
        std::cout << "\n";*/
        influenced->updateLeftBoundaryLevelPointsEpsilonCoefficients(influenced_epsilons);
    }



}


bool
MSRoute::updateVisualizationLeftBoundary() {

    if (!isLeftBoundaryVisualized()) {
        return false;
    }
    PositionVector pShape;

    
    if (visualization_update_remaining_steps <= 0) {
        
        if (visualization_update_remaining_steps == 0) {
           
            //consider numerical errors due to floating point arithmetic. Make the visualized value be equal to the actual one after the last step
            for (size_t i = 0; i < leftBoundaryLevelPointsEpsilonCoefficients.size(); i++) {
                // update the visualized left boundary
                
                leftBoundaryLevelPointsEpsilonCoefficientsVisualized.at(i) = leftBoundaryLevelPointsEpsilonCoefficients.at(i);
            }
            
            getBoundaryShape(leftBoundaryLevelPointsEpsilonCoefficientsVisualized, leftBoundaryOffsets, leftBoundarySlopes, leftBoundaryVisualizationStep, pShape);
            
            ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();
            
            // shape was added already
            shapeCont.reshapePolygon("leftBoundary_" + getID(), pShape);
        }
        return false;
    }


    for (size_t i = 0; i < leftBoundaryLevelPointsEpsilonCoefficients.size(); i++) {
        // update the visualized left boundary
        leftBoundaryLevelPointsEpsilonCoefficientsVisualized.at(i) = leftBoundaryLevelPointsEpsilonCoefficientsVisualized.at(i) + leftBoundaryLevelPointsEpsilonCoefficientsStep.at(i);
    }

    getBoundaryShape(leftBoundaryLevelPointsEpsilonCoefficientsVisualized, leftBoundaryOffsets, leftBoundarySlopes, leftBoundaryVisualizationStep, pShape);

    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();

    // shape was added already
    shapeCont.reshapePolygon("leftBoundary_" + getID(), pShape);


    visualization_update_remaining_steps--;


    // TODO add influencers to be visualized as well

    for (MSRoute* influenced : influencedRoutes) {
        influenced->updateVisualizationLeftBoundary();
    }

    return true;
}


void 
MSRoute::setRightBoundary(std::string& rightBoundaryLevelPointsString, std::string& rightBoundarySlopesString, std::string& rightBoundaryOffsetsString, std::string& rightBoundaryConstant, std::string& visualizeRightBoundaryColor, std::string& vizualizeRightBoundaryStep, std::string& vizualizeRightBoundaryLineWidth) {
    hasRightBoundary = true;

    // use StringTokenizer to get the string xml value as a vector of string values
    StringTokenizer rbpoints(rightBoundaryLevelPointsString);
    std::vector<std::string> rbpointsStringVector = rbpoints.getVector();

    StringTokenizer rbslopes(rightBoundarySlopesString);
    std::vector<std::string> rbslopesStringVector = rbslopes.getVector();

    StringTokenizer rboffsets(rightBoundaryOffsetsString);
    std::vector<std::string> rboffsetsStringVector = rboffsets.getVector();

    if (rbslopesStringVector.size() != rboffsetsStringVector.size()) {// || (rbpointsStringVector.size() - 1) != rbslopesStringVector.size()) {
        std::cout << "Error when parsing Right Boundary for Route:" << getID() << ".";
        std::cout << "Size of rightBoundarySlopes does not match rightBoundaryOffsets.\n";
        hasRightBoundary = false;
        return;
    }

    if ((rbpointsStringVector.size() - 1) != rbslopesStringVector.size()) {
        std::cout << "Error when parsing Right Boundary for Route:" << getID() << ".";
        std::cout << "Size of rightBoundaryLevelPoints does not match rightBoundaryOffsets.\n";
        hasRightBoundary = false;
        return;
    }

    if (rbpointsStringVector.size() == 0 || rbslopesStringVector.size() == 0 || rboffsetsStringVector.size() == 0) {
        std::cout << "Error when parsing Right Boundary for Route:" << getID() << ".";
        std::cout << "At least one of the associated elements is empty.\n";
        hasRightBoundary = false;
        return;
    }

    // convert the string elements to double
    double element_value, min_element{ 0 }, max_element{ 0 };

    int points_size = rbpointsStringVector.size();
    for (int i = 0; i < points_size; i++) {

        element_value = StringUtils::toDouble(rbpointsStringVector.at(i));
        rightBoundaryLevelPoints.push_back(element_value);
        if (element_value > max_element || i == 0) {
            max_element = element_value;
        }
        if (element_value < min_element || i == 0) {
            min_element = element_value;
        }
        if (i == points_size - 1) {
            continue; // or break; It is the last iteration so they are equivalent
        }

        element_value = StringUtils::toDouble(rbslopesStringVector.at(i));
        rightBoundarySlopes.push_back(element_value);

        element_value = StringUtils::toDouble(rboffsetsStringVector.at(i));
        rightBoundaryOffsets.push_back(element_value);

    }

    if (rightBoundaryConstant == "empty") {
        const std::vector<MSLane*> myLanes = myEdges.at(0)->getLanes();
        if (myLanes.size() == 0) {
            std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty lane set for first edge!\n";
            return;
        }
        MSLane* myFirstLane{ myLanes.at(0) };
        double pathAngle = myFirstLane->getShape().angleAt2D(0);
        bool direction = cos(pathAngle) > 0;
        rightBoundaryConstantLevelPoint = direction ? min_element : max_element;
    }
    else {
        element_value = StringUtils::toDouble(rightBoundaryConstant);
        rightBoundaryConstantLevelPoint = element_value;
        

    }

    // visualization is not specified
    if (visualizeRightBoundaryColor.empty() || visualizeRightBoundaryColor == "") {
        
        return;
    }
    

    const std::vector<MSLane*> myLanes = myEdges.at(0)->getLanes();
    if (myLanes.size() == 0) {
        std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty lane set for first edge!\n";
        return;
    }
    MSLane* myFirstLane{ myLanes.at(0) };


    // in global coordinates
    double x_origin = myFirstLane->getShape()[0].x();

    // find x position of the route's end in global coordinates. It will be the last point of the last lane's shape
    const std::vector<MSLane*> myLanes_last = myEdges.at(myEdges.size() - 1)->getLanes();
    MSLane* myLastLane{ myLanes_last.at(myLanes_last.size() - 1) };
    double x_end = myLastLane->getShape()[myLastLane->getShape().size() - 1].x();

    // by default we select a step that will create visualization segments of at least twice the amount of offset points
    double rightBoundaryVisualizationStep = abs(x_end - x_origin) / (8 * rightBoundaryOffsets.size());

    if ((!vizualizeRightBoundaryStep.empty()) && vizualizeRightBoundaryStep != "") {
        double step_parse = StringUtils::toDouble(vizualizeRightBoundaryStep);

        if (step_parse > rightBoundaryVisualizationStep) {
            std::cout << "Warning! Step selected for the vizualisation of left boundary at route " << getID() << " will be overwritten! We want the step selection to provide at least twice the number of offset points!\n";
        }
        else {
            rightBoundaryVisualizationStep = step_parse;
        }

    }

    // default value
    double vizualizeBoundaryLineWidth = 0.1;
    if ((!vizualizeRightBoundaryLineWidth.empty()) && vizualizeRightBoundaryLineWidth != "") {
        double line_width = StringUtils::toDouble(vizualizeRightBoundaryLineWidth);
        if (line_width <= 0) {
            std::cout << "Line width for left boundary at route " << getID() << " cannot be non-positive! Default value of 0.1 will be selected instead.\n";
        }
        else {
            vizualizeBoundaryLineWidth = line_width;
        }
    }


    




    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();
    PositionVector pShape;



    getBoundaryShape(rightBoundaryLevelPoints, rightBoundaryOffsets, rightBoundarySlopes, rightBoundaryVisualizationStep, pShape);

    RGBColor col = RGBColor::parseColor(visualizeRightBoundaryColor);

    if (!shapeCont.addPolygon("rightBoundary_" + getID(), "rightBoundary", col, 130., Shape::DEFAULT_ANGLE, Shape::DEFAULT_IMG_FILE, Shape::DEFAULT_RELATIVEPATH, pShape, false, false, vizualizeBoundaryLineWidth)) {
        std::cout << "Error! could not visualize right boundary of route:" << getID() << "\n";
    }


    // we also need to update the veh out of bounds event function
  

}

size_t MSRoute::addInfluencedRoute(MSRoute* influenced, std::vector<double>& influencedLeftBoundaryOffsets) {


    size_t my_idx = 0, influenced_idx = 0;
    int epsilon_start_idx = -1, epsilon_end_idx = -1;
    int epsilon_influencer_start_idx, epsilon_influencer_end_idx;
    // we choose the path direction according to the first lane on the path
    const std::vector<MSLane*> myLanes = myEdges.at(0)->getLanes();
    if (myLanes.size() == 0) {
        std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty lane set for first edge!\n";
        return 0;
    }
    MSLane* myFirstLane{ myLanes.at(0) };
    double pathAngle = myFirstLane->getShape().angleAt2D(0);
    bool direction = cos(pathAngle) > 0;

    while ((my_idx <= leftBoundaryOffsets.size()-1) && (influenced_idx <= influencedLeftBoundaryOffsets.size()-1)) {
        
        if (leftBoundaryOffsets[my_idx] == influencedLeftBoundaryOffsets[influenced_idx]) {
            if (epsilon_start_idx == -1) {
                // epsilons control the level points, so we should start from the level point after the first occurence
                epsilon_start_idx = influenced_idx+1;
                epsilon_influencer_start_idx = my_idx+1;
            }
            my_idx++;
            influenced_idx++;
        }
        else if(epsilon_start_idx != -1) {
            // we stop finding common boundary offsets, therefore the region that is common among influencer and influenced is complete
            epsilon_end_idx = influenced_idx;
            epsilon_influencer_end_idx = my_idx;
            break;
        }
        else if (leftBoundaryOffsets[my_idx] < influencedLeftBoundaryOffsets[influenced_idx]) {
            // left to right direction
            if (direction) {
                my_idx++;
            }
            else {
                influenced_idx++;
            }
            
        }
        else if (leftBoundaryOffsets[my_idx] > influencedLeftBoundaryOffsets[influenced_idx]) {
            // left to right direction
            if (direction) {
                influenced_idx++;
            }
            else {
                my_idx++;                
            }
        }
        else {
            std::cout << "Error at obtaining influenced path's common epsilons! we shouldn't reach this point!\n";
            return 0;
        }
    }

    if (epsilon_start_idx == -1) {
        std::cout << "Error at obtaining influenced path's "<< influenced->getID()<<" common epsilons! Offset values do not coincide with influencer's"<<getID()<<"!\n";
        std::cout << "Start:" << epsilon_start_idx;
        return 0;
    }

    if (epsilon_end_idx == -1) {
        // means that up to the last element we had equality of terms
        epsilon_end_idx = influencedLeftBoundaryOffsets.size();
        epsilon_influencer_end_idx = leftBoundaryOffsets.size();
    }

    influencedRoutes.push_back(influenced);
    influencedRoutesEpsilonIndicesStartEnd.push_back(std::make_pair((size_t)epsilon_start_idx, (size_t)epsilon_end_idx));
    influencedRoutesInfluencerEpsilonIndicesStartEnd.push_back(std::make_pair((size_t)epsilon_influencer_start_idx, (size_t)epsilon_influencer_end_idx));

    return influencedRoutes.size();

}
// LFPlugin End


MSRoute::~MSRoute() {
    delete myColor;
}


MSRouteIterator
MSRoute::begin() const {
    return myEdges.begin();
}


MSRouteIterator
MSRoute::end() const {
    return myEdges.end();
}


int
MSRoute::size() const {
    return (int)myEdges.size();
}


const MSEdge*
MSRoute::getLastEdge() const {
    assert(myEdges.size() > 0);
    return myEdges[myEdges.size() - 1];
}


void
MSRoute::addReference() const {
    myReferenceCounter++;
}


void
MSRoute::release() const {
    myReferenceCounter--;
    if (myReferenceCounter == 0) {
#ifdef HAVE_FOX
        FXMutexLock f(myDictMutex);
#endif
        myDict.erase(myID);
        delete this;
    }
}


bool
MSRoute::dictionary(const std::string& id, const MSRoute* route) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    if (myDict.find(id) == myDict.end() && myDistDict.find(id) == myDistDict.end()) {
        myDict[id] = route;
        return true;
    }
    return false;
}


bool
MSRoute::dictionary(const std::string& id, RandomDistributor<const MSRoute*>* const routeDist, const bool permanent) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    if (myDict.find(id) == myDict.end() && myDistDict.find(id) == myDistDict.end()) {
        myDistDict[id] = std::make_pair(routeDist, permanent);
        return true;
    }
    return false;
}


const MSRoute*
MSRoute::dictionary(const std::string& id, std::mt19937* rng) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    RouteDict::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        RouteDistDict::iterator it2 = myDistDict.find(id);
        if (it2 == myDistDict.end() || it2->second.first->getOverallProb() == 0) {
            return nullptr;
        }
        return it2->second.first->get(rng);
    }
    return it->second;
}


bool
MSRoute::hasRoute(const std::string& id) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    return myDict.find(id) != myDict.end();
}


RandomDistributor<const MSRoute*>*
MSRoute::distDictionary(const std::string& id) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    RouteDistDict::iterator it2 = myDistDict.find(id);
    if (it2 == myDistDict.end()) {
        return nullptr;
    }
    return it2->second.first;
}


void
MSRoute::clear() {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    for (RouteDistDict::iterator i = myDistDict.begin(); i != myDistDict.end(); ++i) {
        delete i->second.first;
    }
    myDistDict.clear();
    for (RouteDict::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        delete i->second;
    }
    myDict.clear();
}


void
MSRoute::checkDist(const std::string& id) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    RouteDistDict::iterator it = myDistDict.find(id);
    if (it != myDistDict.end() && !it->second.second) {
        const std::vector<const MSRoute*>& routes = it->second.first->getVals();
        for (std::vector<const MSRoute*>::const_iterator i = routes.begin(); i != routes.end(); ++i) {
            (*i)->release();
        }
        delete it->second.first;
        myDistDict.erase(it);
    }
}


void
MSRoute::insertIDs(std::vector<std::string>& into) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    into.reserve(myDict.size() + myDistDict.size() + into.size());
    for (RouteDict::const_iterator i = myDict.begin(); i != myDict.end(); ++i) {
        into.push_back((*i).first);
    }
    for (RouteDistDict::const_iterator i = myDistDict.begin(); i != myDistDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


int
MSRoute::writeEdgeIDs(OutputDevice& os, const MSEdge* const from, const MSEdge* const upTo) const {
    int numWritten = 0;
    ConstMSEdgeVector::const_iterator i = myEdges.begin();
    if (from != nullptr) {
        i = std::find(myEdges.begin(), myEdges.end(), from);
    }
    for (; i != myEdges.end(); ++i) {
        if ((*i) == upTo) {
            return numWritten;
        }
        os << (*i)->getID();
        numWritten++;
        if (upTo || i != myEdges.end() - 1) {
            os << ' ';
        }
    }
    return numWritten;
}


bool
MSRoute::containsAnyOf(const MSEdgeVector& edgelist) const {
    MSEdgeVector::const_iterator i = edgelist.begin();
    for (; i != edgelist.end(); ++i) {
        if (contains(*i)) {
            return true;
        }
    }
    return false;
}


const MSEdge*
MSRoute::operator[](int index) const {
    return myEdges[index];
}


void
MSRoute::dict_saveState(OutputDevice& out) {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    for (RouteDict::iterator it = myDict.begin(); it != myDict.end(); ++it) {
        out.openTag(SUMO_TAG_ROUTE).writeAttr(SUMO_ATTR_ID, (*it).second->getID());
        out.writeAttr(SUMO_ATTR_STATE, (*it).second->myAmPermanent);
        out.writeAttr(SUMO_ATTR_EDGES, (*it).second->myEdges).closeTag();
    }
    for (const auto& item : myDistDict) {
        if (item.second.first->getVals().size() > 0) {
            out.openTag(SUMO_TAG_ROUTE_DISTRIBUTION).writeAttr(SUMO_ATTR_ID, item.first);
            out.writeAttr(SUMO_ATTR_STATE, item.second.second);
            out.writeAttr(SUMO_ATTR_ROUTES, item.second.first->getVals());
            out.writeAttr(SUMO_ATTR_PROBS, item.second.first->getProbs());
            out.closeTag();
        }
    }
}

void
MSRoute::dict_clearState() {
#ifdef HAVE_FOX
    FXMutexLock f(myDictMutex);
#endif
    for (auto item : myDict) {
        delete item.second;
    }
    myDistDict.clear();
    myDict.clear();
}


double
MSRoute::getDistanceBetween(double fromPos, double toPos,
                            const MSEdge* fromEdge, const MSEdge* toEdge, bool includeInternal, int routePosition) const {
    //std::cout << SIMTIME << " getDistanceBetween from=" << fromEdge->getID() << " to=" << toEdge->getID() << " fromPos=" << fromPos << " toPos=" << toPos << " includeInternal=" << includeInternal << "\n";
    if (routePosition < 0 || routePosition >= (int)myEdges.size()) {
        throw ProcessError("Invalid routePosition " + toString(routePosition) + " for route with " + toString(myEdges.size()) + " edges");
    }
    if (fromEdge->isInternal() && toEdge->isInternal() && fromEdge->getToJunction() == toEdge->getToJunction()) {
        // internal edges within the same junction
        if (fromEdge == toEdge) {
            if (fromPos <= toPos) {
                return toPos - fromPos;
            }
        } else if (fromEdge->getSuccessors().front() == toEdge) {
            return fromEdge->getLength() - fromPos + toPos;
        }
    }
    if (fromEdge->isInternal()) {
        if (fromEdge == myEdges.front()) {
            const MSEdge* succ = fromEdge->getSuccessors().front();
            assert(succ != 0);
            //std::cout << "  recurse fromSucc=" << succ->getID() << "\n";
            return (fromEdge->getLength() - fromPos) + getDistanceBetween(0, toPos, succ, toEdge, includeInternal);
        } else {
            const MSEdge* pred = fromEdge->getPredecessors().front();
            assert(pred != 0);
            //std::cout << "  recurse fromPred=" << pred->getID() << "\n";
            return getDistanceBetween(pred->getLength(), toPos, pred, toEdge, includeInternal, routePosition) - fromPos;
        }
    }
    if (toEdge->isInternal()) {
        const MSEdge* pred = toEdge->getPredecessors().front();
        assert(pred != 0);
        //std::cout << "  recurse toPred=" << pred->getID() << "\n";
        return toPos + getDistanceBetween(fromPos, pred->getLength(), fromEdge, pred, includeInternal, routePosition);
    }
    ConstMSEdgeVector::const_iterator it = std::find(myEdges.begin() + routePosition, myEdges.end(), fromEdge);
    if (it == myEdges.end() || std::find(it, myEdges.end(), toEdge) == myEdges.end()) {
        // start or destination not contained in route
        return std::numeric_limits<double>::max();
    }
    ConstMSEdgeVector::const_iterator it2 = std::find(it + 1, myEdges.end(), toEdge);

    if (fromEdge == toEdge) {
        if (fromPos <= toPos) {
            return toPos - fromPos;
        } else if (it2 == myEdges.end()) {
            // we don't visit the edge again
            return std::numeric_limits<double>::max();
        }
    }
    return getDistanceBetween(fromPos, toPos, it, it2, includeInternal);
}


double
MSRoute::getDistanceBetween(double fromPos, double toPos,
                            const MSRouteIterator& fromEdge, const MSRouteIterator& toEdge, bool includeInternal) const {
    bool isFirstIteration = true;
    double distance = -fromPos;
    MSRouteIterator it = fromEdge;
    if (fromEdge == toEdge) {
        // destination position is on start edge
        if (fromPos <= toPos) {
            return toPos - fromPos;
        } else {
            // we cannot go backwards. Something is wrong here
            return std::numeric_limits<double>::max();
        }
    } else if (fromEdge > toEdge) {
        // we don't visit the edge again
        return std::numeric_limits<double>::max();
    }
    for (; it != end(); ++it) {
        if (it == toEdge && !isFirstIteration) {
            distance += toPos;
            break;
        } else {
            distance += (*it)->getLength();
            if (includeInternal && (it + 1) != end()) {
                distance += (*it)->getInternalFollowingLengthTo(*(it + 1));
            }
        }
        isFirstIteration = false;
    }
    return distance;
}


const RGBColor&
MSRoute::getColor() const {
    if (myColor == nullptr) {
        return RGBColor::DEFAULT_COLOR;
    }
    return *myColor;
}


const std::vector<SUMOVehicleParameter::Stop>&
MSRoute::getStops() const {
    return myStops;
}


/****************************************************************************/
