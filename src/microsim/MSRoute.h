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
/// @file    MSRoute.h
/// @author  Daniel Krajzewicz
/// @author  Friedemann Wesner
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Sept 2002
///
// A vehicle route
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <utils/common/Named.h>
#include <utils/distribution/RandomDistributor.h>
#include <utils/common/RGBColor.h>
#include <utils/vehicle/SUMOVehicleParameter.h>
#include <utils/common/Parameterised.h>
#ifdef HAVE_FOX
#include <fx.h>
#include <FXThread.h>
#endif

// LFPlugin Begin
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>
// LFPlugin End

// ===========================================================================
// class declarations
// ===========================================================================
class MSEdge;
class OutputDevice;


// ===========================================================================
// types definitions
// ===========================================================================
typedef std::vector<const MSEdge*> ConstMSEdgeVector;
typedef std::vector<MSEdge*> MSEdgeVector;
typedef ConstMSEdgeVector::const_iterator MSRouteIterator;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSRoute
 */
class MSRoute : public Named, public Parameterised {
public:
    /// Constructor
    MSRoute(const std::string& id, const ConstMSEdgeVector& edges,
            const bool isPermanent, const RGBColor* const c,
            const std::vector<SUMOVehicleParameter::Stop>& stops);

    /// Destructor
    virtual ~MSRoute();

    /// Returns the begin of the list of edges to pass
    MSRouteIterator begin() const;

    /// Returns the end of the list of edges to pass
    MSRouteIterator end() const;

    /// Returns the number of edges to pass
    int size() const;

    /// returns the destination edge
    const MSEdge* getLastEdge() const;

    /** @brief increments the reference counter for the route */
    void addReference() const;

    /** @brief deletes the route if there are no further references to it*/
    void release() const;

    /** @brief Output the edge ids up to but not including the id of the given edge
     * @param[in] os The stream to write the routes into (binary)
     * @param[in] from The first edge to be written
     * @param[in] upTo The first edge that shall not be written
     * @return The number of edges written
     */
    int writeEdgeIDs(OutputDevice& os, const MSEdge* const from, const MSEdge* const upTo = 0) const;

    bool contains(const MSEdge* const edge) const {
        return std::find(myEdges.begin(), myEdges.end(), edge) != myEdges.end();
    }

    // LFPlugin Begin
    //check where the edge is
    int edge_index(const MSEdge* const edge) const {
        ConstMSEdgeVector::const_iterator it = std::find(myEdgeswInternal.begin(), myEdgeswInternal.end(), edge);
        if (it == myEdgeswInternal.end()) {
            return -1;
        }
        return (int)(it - myEdgeswInternal.begin());
    }

    int edge_index_bike(const MSEdge* const edge) const {
        ConstMSEdgeVector::const_iterator it = std::find(myEdgeswInternalBike.begin(), myEdgeswInternalBike.end(), edge);
        if (it == myEdgeswInternalBike.end()) {
            return -1;
        }
        return (int)(it - myEdgeswInternalBike.begin());
    }

    int edge_index_normal(const MSEdge* const edge) const {
        ConstMSEdgeVector::const_iterator it = std::find(myEdges.begin(), myEdges.end(), edge);
        if (it == myEdges.end()) {
            return -1;
        }
        return (int)(it - myEdges.begin());
    }

    const ConstMSEdgeVector& getEdgeswInternal() const {
        return myEdgeswInternal;
    }

    const ConstMSEdgeVector& getEdgeswInternalBike() const {
        return myEdgeswInternalBike;
    }

    void getBoundaryShape(std::vector<double>& boundaryLevelPoints, std::vector<double>& boundaryOffsets, std::vector<double>& boundarySlopes, double step, PositionVector& boundaryShape);


    void setLeftBoundary(std::string& leftBoundaryLevelPointsString, std::string& leftBoundarySlopesString, std::string& leftBoundaryOffsetsString, std::string& influencedBy, std::string& visualizeLeftBoundaryColor, std::string& vizualizeLeftBoundaryStep, std::string& vizualizeLeftBoundaryLineWidth, std::string& vizualizeLeftBoundaryUpdateDelaySeconds);


    void setRightBoundary(std::string& rightBoundaryLevelPointsString, std::string& rightBoundarySlopesString, std::string& rightBoundaryOffsetsString, std::string& rightBoundaryConstant, std::string& visualizeRightBoundaryColor, std::string& vizualizeRightBoundaryStep, std::string& vizualizeRightBoundaryLineWidth);

    
    

    const std::vector<double>& getLeftBoundaryLevelPoints() const {
        
        return leftBoundaryLevelPoints;
    }

    const std::vector<double>& getLeftBoundaryLevelPointsWithEpsilon() const {

        return leftBoundaryLevelPointsEpsilonCoefficients;
    }

    const bool isLeftBoundaryVisualized() const {

        return visualizeLeftBoundary;
    }

    // returns the left boundary offsets, according to the epsilon values selected, results will be appended in the input argument
    void updateLeftBoundaryLevelPointsEpsilonCoefficients(std::vector<double>& leftBoundaryEpsilons);

    // returns false when the visualized boundary is aligned with the updated one. If further change is required, it returns true
    bool updateVisualizationLeftBoundary();

    const std::vector<double>& getLeftBoundarySlopes() const {
        return leftBoundarySlopes;
    }

    const std::vector<double>& getLeftBoundaryOffsets() const {
        return leftBoundaryOffsets;
    }

    const std::vector<double>& getRightBoundaryLevelPoints() const {
        return rightBoundaryLevelPoints;
    }

    const std::vector<double>& getRightBoundarySlopes() const {
        return rightBoundarySlopes;
    }

    const std::vector<double>& getRightBoundaryOffsets() const {
        return rightBoundaryOffsets;
    }

    const std::vector<std::pair<long long, double>>& getLeftBoundaryOffsetsLocalPositions() const {
        return localEdgePosLeftBoundaryOffsets;
    }

    size_t addInfluencedRoute(MSRoute* influenced, std::vector<double>& influencedLeftBoundaryOffsets);

    int getVisualizerUpdateDelaySteps() const {
        return num_of_steps_visualization_update_delay;
    }

    // LFPlugin End

    bool containsAnyOf(const MSEdgeVector& edgelist) const;

    const MSEdge* operator[](int index) const;

    /// @name State I/O
    /// @{

    /** @brief Saves all known routes into the given stream
     *
     * @param[in] os The stream to write the routes into (binary)
     */
    static void dict_saveState(OutputDevice& out);

    /** @brief Decrement  all route references before quick-loading state */
    static void dict_clearState();
    /// @}

    const ConstMSEdgeVector& getEdges() const {
        return myEdges;
    }

    /** @brief Compute the distance between 2 given edges on this route, including the length of internal lanes.
     * Note, that for edges which contain loops:
     * - the first occurance of fromEdge will be used
     * - the first occurance of toEdge after the first occurance of fromEdge will be used
     *
     * @param[in] fromPos  position on the first edge, at wich the computed distance begins
     * @param[in] toPos    position on the last edge, at which the coumputed distance endsance
     * @param[in] fromEdge edge at wich computation begins
     * @param[in] toEdge   edge at which distance computation shall stop
     * @param[in] includeInternal Whether the lengths of internal edges shall be counted
     * @param[in] routePosition Optional offset when searching for the fromEdge within the route
     * @return             distance between the position fromPos on fromEdge and toPos on toEdge
     */
    double getDistanceBetween(double fromPos, double toPos, const MSEdge* fromEdge, const MSEdge* toEdge, bool includeInternal = true, int routePosition = 0) const;

    /** @brief Compute the distance between 2 given edges on this route, including the length of internal lanes.
     * This has the same semantics as above but uses iterators instead of edge
     * points so looping routes are not an issue.
     *
     * @param[in] fromPos  position on the first edge, at wich the computed distance begins
     * @param[in] toPos    position on the last edge, at which the coumputed distance endsance
     * @param[in] fromEdge edge at wich computation begins
     * @param[in] toEdge   edge at which distance computation shall stop
     * @param[in] includeInternal Whether the lengths of internal edges shall be counted
     * @return             distance between the position fromPos on fromEdge and toPos on toEdge
     */
    double getDistanceBetween(double fromPos, double toPos, const MSRouteIterator& fromEdge, const MSRouteIterator& toEdge, bool includeInternal = true) const;

    /// @brief Returns the color
    const RGBColor& getColor() const;

    /// @brief returns the period
    SUMOTime getPeriod() const {
        return myPeriod;
    }

    /** @brief Returns the costs of the route
     *
     * @return The route's costs (normally the time needed to pass it)
     */
    double getCosts() const {
        return myCosts;
    }

    /** @brief Returns the estimated savings due to using this route (compare to the route before rerouting)
     *
     * @return The route's estimated savings (the difference in costs of this route to the previous one)
     */
    double getSavings() const {
        return mySavings;
    }

    /// @brief sets the period
    void setPeriod(SUMOTime period) {
        myPeriod = period;
    }

    /** @brief Sets the costs of the route
     *
     * @param[in] costs The new route costs
     */
    void setCosts(double costs) {
        myCosts = costs;
    }
    /** @brief Sets the savings of the route
     *
     * @param[in] costs The new route costs
     */
    void setSavings(double savings) {
        mySavings = savings;
    }

    bool mustReroute() const {
        return myReroute;
    }

    void setReroute(bool reroute = true) {
        myReroute = reroute;
    }

    /// Returns the stops
    const std::vector<SUMOVehicleParameter::Stop>& getStops() const;

public:
    /** @brief Adds a route to the dictionary.
     *
     *  Returns true if the route could be added,
     *  false if a route (distribution) with the same name already exists.
     *
     * @param[in] id    the id for the new route
     * @param[in] route pointer to the route object
     * @return          whether adding was successful
     */
    static bool dictionary(const std::string& id, const MSRoute* route);

    /** @brief Adds a route distribution to the dictionary.
     *
     *  Returns true if the distribution could be added,
     *  false if a route (distribution) with the same name already exists.
     *
     * @param[in] id         the id for the new route distribution
     * @param[in] routeDist  pointer to the distribution object
     * @param[in] permanent  whether the new route distribution survives more than one vehicle / flow
     * @return               whether adding was successful
     */
    static bool dictionary(const std::string& id, RandomDistributor<const MSRoute*>* const routeDist, const bool permanent = true);

    /** @brief Returns the named route or a sample from the named distribution.
     *
     *  Returns 0 if no route and no distribution with the given name exists
     *  or if the distribution exists and is empty.
     *
     * @param[in] id    the id of the route or the distribution
     * @return          the route (sample)
     */
    static const MSRoute* dictionary(const std::string& id, std::mt19937* rng = 0);

    /// @brief returns whether a route with the given id exists
    static bool hasRoute(const std::string& id);

    /** @brief Returns the named route distribution.
     *
     *  Returns 0 if no route distribution with the given name exists.
     *
     * @param[in] id    the id of the route distribution
     * @return          the route distribution
     */
    static RandomDistributor<const MSRoute*>* distDictionary(const std::string& id);

    /// Clears the dictionary (delete all known routes, too)
    static void clear();

    /// Checks the distribution whether it is permanent and deletes it if not
    static void checkDist(const std::string& id);

    static void insertIDs(std::vector<std::string>& into);

private:
    /// The list of edges to pass
    ConstMSEdgeVector myEdges;

    // LFPlugin Begin
    // additional vector that contains all edges along with the internal ones
    ConstMSEdgeVector myEdgeswInternal;
    ConstMSEdgeVector myEdgeswInternalBike;
    bool hasLeftBoundary;
    std::vector<double> leftBoundaryLevelPoints;
    std::vector<double> leftBoundarySlopes;
    std::vector<double> leftBoundaryOffsets;
    
    double leftBoundaryVisualizationStep;
    bool visualizeLeftBoundary;

    bool hasRightBoundary;
    std::vector<double> rightBoundaryLevelPoints;
    std::vector<double> rightBoundarySlopes;
    std::vector<double> rightBoundaryOffsets;
    
    double rightBoundaryConstantLevelPoint; // this is useful for the use of epsilon
    std::vector<double> leftBoundaryLevelPointsEpsilonCoefficients;
    std::vector<double> leftBoundaryLevelPointsEpsilonCoefficientsVisualized;
    std::vector<double> leftBoundaryLevelPointsEpsilonCoefficientsStep;
    int num_of_steps_visualization_update_delay;
    int visualization_update_remaining_steps;

    std::vector<std::pair<long long, double>> localEdgePosLeftBoundaryOffsets;
    std::vector<MSRoute*> influencedRoutes;
    std::vector<std::pair<size_t,size_t>> influencedRoutesEpsilonIndicesStartEnd;
    std::vector<std::pair<size_t, size_t>> influencedRoutesInfluencerEpsilonIndicesStartEnd;
    // LFPlugin End

    /// whether the route may be deleted after the last vehicle abandoned it
    const bool myAmPermanent;

    /// Information by how many vehicles the route is used
    mutable int myReferenceCounter;

    /// The color
    const RGBColor* const myColor;

    /// The period when repeating this route
    SUMOTime myPeriod;

    /// @brief The assigned or calculated costs
    double myCosts;

    /// @brief The estimated savings when rerouting
    double mySavings;

    /// @brief Whether this route is incomplete and requires rerouting
    bool myReroute;

    /// @brief List of the stops on the parsed route
    std::vector<SUMOVehicleParameter::Stop> myStops;

private:
    /// Definition of the dictionary container
    typedef std::map<std::string, const MSRoute*> RouteDict;

    /// The dictionary container
    static RouteDict myDict;

    /// Definition of the dictionary container
    typedef std::map<std::string, std::pair<RandomDistributor<const MSRoute*>*, bool> > RouteDistDict;

    /// The dictionary container
    static RouteDistDict myDistDict;

#ifdef HAVE_FOX
    /// @brief the mutex for the route dictionaries
    static FXMutex myDictMutex;
#endif
private:
    /** invalid assignment operator */
    MSRoute& operator=(const MSRoute& s);

};
