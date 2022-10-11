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
    myStops(stops)
    // LFPlugin Begin
    ,
    hasLeftBoundary(false),
    hasRightBoundary(false)    
    // LFPlugin End
    {
    // LFPlugin Begin
    // initialize myEdgeswInternal
    const MSEdge* tmp_edge_prev = nullptr;
    const MSEdge* tmp_edge_internal;
    for (ConstMSEdgeVector::iterator it = myEdges.begin(); it != myEdges.end(); it++) {
        
        if (tmp_edge_prev != nullptr) {
            tmp_edge_internal = tmp_edge_prev->getInternalFollowingEdge((*it));
            if (tmp_edge_internal != nullptr) {
                myEdgeswInternal.push_back(tmp_edge_internal);
            }
        }
        myEdgeswInternal.push_back((*it));
        
        

        tmp_edge_prev = (*it);
    }
    // LFPlugin End
    }

// LFPlugin Begin

void 
MSRoute::setLeftBoundary(std::string& leftBoundaryLevelPointsString, std::string& leftBoundarySlopesString, std::string& leftBoundaryOffsetsString) {
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
        
        if (i == points_size - 1) {
            continue; // or break; It is the last iteration so they are equivalent
        }

        element_value = StringUtils::toDouble(lbslopesStringVector.at(i));
        leftBoundarySlopes.push_back(element_value);

        element_value = StringUtils::toDouble(lboffsetsStringVector.at(i));
        leftBoundaryOffsets.push_back(element_value);

        
    }
    
    
}

void
MSRoute::updateLeftBoundaryLevelPointsEpsilonCoefficients(std::vector<double>& leftBoundaryEpsilons){
    
    double boundaries_width;
    
    for (int i = 0; i < leftBoundaryLevelPoints.size(); i++) {
        boundaries_width = leftBoundaryLevelPoints.at(i) - rightBoundaryConstantLevelPoint;
        leftBoundaryLevelPointsEpsilonCoefficients.at(i) = rightBoundaryConstantLevelPoint + boundaries_width * leftBoundaryEpsilons.at(i);
    }

}

void 
MSRoute::setRightBoundary(std::string& rightBoundaryLevelPointsString, std::string& rightBoundarySlopesString, std::string& rightBoundaryOffsetsString) {
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


    // initialize the right Boundary level that is taken for the internal boundary control through the epsilon values
    // Considering it is used for scenarios with on-ramps and off-ramps. It makes sense to pick the level point associated with the main highway
    // This will be either the max or min level point depending on the direction
    // We can get direction through the first lane
    if (myEdges.size() == 0) {
        std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty path!\n";
        return;
    }

    // we choose the path direction according to the first lane on the path
    const std::vector<MSLane*> myLanes = myEdges.at(0)->getLanes();
    if (myLanes.size() == 0) {
        std::cout << "Error initializing the right boundary constant value for internal boundary control. Empty lane set for first edge!\n";
        return;
    }
    MSLane* myFirstLane{ myLanes.at(0) };
    double pathAngle = myFirstLane->getShape().angleAt2D(0);

    if (cos(pathAngle) > 0) {
        rightBoundaryConstantLevelPoint = max_element;
    }
    else {
        rightBoundaryConstantLevelPoint = min_element;
    }
    if (cos(pathAngle) != 1 && cos(pathAngle) != -1) {
        std::cout << "Warning! Constant right boundary value for internal control was set for left or right direction, but the path's angle is: "<< pathAngle << " rad \n";
    }

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
