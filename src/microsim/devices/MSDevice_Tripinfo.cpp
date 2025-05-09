/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2009-2020 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Tripinfo.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 30.01.2009
///
// A device which collects info on the vehicle trip
/****************************************************************************/
#include <config.h>

#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <mesosim/MEVehicle.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include "MSDevice_Vehroutes.h"
#include "MSDevice_Tripinfo.h"
// LFPlugin Begin
#include <microsim/cfmodels/MSCFModel_LaneFree.h>
// LFPlugin End

#define NOT_ARRIVED TIME2STEPS(-1)


// ===========================================================================
// static members
// ===========================================================================
std::set<const MSDevice_Tripinfo*, ComparatorNumericalIdLess> MSDevice_Tripinfo::myPendingOutput;

double MSDevice_Tripinfo::myVehicleCount(0);
double MSDevice_Tripinfo::myTotalRouteLength(0);
double MSDevice_Tripinfo::myTotalSpeed(0);
SUMOTime MSDevice_Tripinfo::myTotalDuration(0);
SUMOTime MSDevice_Tripinfo::myTotalWaitingTime(0);
SUMOTime MSDevice_Tripinfo::myTotalTimeLoss(0);
SUMOTime MSDevice_Tripinfo::myTotalDepartDelay(0);
SUMOTime MSDevice_Tripinfo::myWaitingDepartDelay(-1);

// LFPlugin Begin
SUMOTime MSDevice_Tripinfo::myTotalTimeLossNoNeg(0);
SUMOTime MSDevice_Tripinfo::myTotalTimeLossOnlyNeg(0);
SUMOTime MSDevice_Tripinfo::myTotalTimeLossExcludeEdges(0);
SUMOTime MSDevice_Tripinfo::myTotalTimeLossNoNegExcludeEdges(0);
SUMOTime MSDevice_Tripinfo::myTotalExpectedTime(0);
SUMOTime MSDevice_Tripinfo::myTotalDelayTime(0);
SUMOTime MSDevice_Tripinfo::myTotalDurationNoNeg(0);
long long MSDevice_Tripinfo::myTotalVehicleCount(0);
long long MSDevice_Tripinfo::myTotalVehicleCountExcludeEdges(0);
long long MSDevice_Tripinfo::myVehicleCountNoNeg(0);
long long MSDevice_Tripinfo::myVehicleCountOnlyNeg(0);
//Made for ad-hoc log files on time-delay and tts
//FILE* MSDevice_Tripinfo::fp_time_delay = fopen("time_delay.txt", "w");
//FILE* MSDevice_Tripinfo::fp_time_delay_excluded = fopen("time_delay_excluded.txt", "w");
//FILE* MSDevice_Tripinfo::fp_num_of_vehs = fopen("num_of_vehs.txt", "w");
//FILE* MSDevice_Tripinfo::fp_num_of_vehs_excluded = fopen("num_of_vehs_excluded.txt", "w");
// LFPlugin End
int MSDevice_Tripinfo::myWalkCount(0);
double MSDevice_Tripinfo::myTotalWalkRouteLength(0);
SUMOTime MSDevice_Tripinfo::myTotalWalkDuration(0);
SUMOTime MSDevice_Tripinfo::myTotalWalkTimeLoss(0);
std::vector<int> MSDevice_Tripinfo::myRideCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideBusCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideRailCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideTaxiCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideBikeCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideAbortCount({0, 0});
std::vector<double> MSDevice_Tripinfo::myTotalRideWaitingTime({0., 0.});
std::vector<double> MSDevice_Tripinfo::myTotalRideRouteLength({0., 0.});
std::vector<SUMOTime> MSDevice_Tripinfo::myTotalRideDuration({0, 0});

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Tripinfo::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Tripinfo Device");
    insertDefaultAssignmentOptions("tripinfo", "Tripinfo Device", oc);
}


void
MSDevice_Tripinfo::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    const bool enableByOutputOption = oc.isSet("tripinfo-output") || oc.getBool("duration-log.statistics");
    if (equippedByDefaultAssignmentOptions(oc, "tripinfo", v, enableByOutputOption)) {
        MSDevice_Tripinfo* device = new MSDevice_Tripinfo(v, "tripinfo_" + v.getID());
        into.push_back(device);
        myPendingOutput.insert(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_Tripinfo-methods
// ---------------------------------------------------------------------------
MSDevice_Tripinfo::MSDevice_Tripinfo(SUMOVehicle& holder, const std::string& id) :
    MSVehicleDevice(holder, id),
    myDepartLane(""),
    myDepartSpeed(-1),
    myDepartPosLat(0),
    myWaitingTime(0),
    myAmWaiting(false),
    myWaitingCount(0),
    myStoppingTime(0),
    myParkingStarted(-1),
    myArrivalTime(NOT_ARRIVED),
    myArrivalLane(""),
    myArrivalPos(-1),
    myArrivalPosLat(0.),
    myArrivalSpeed(-1),
    myArrivalReason(MSMoveReminder::NOTIFICATION_ARRIVED),
    myMesoTimeLoss(0),
    myRouteLength(0.) {
}


MSDevice_Tripinfo::~MSDevice_Tripinfo() {
    // ensure clean up for vaporized vehicles which do not generate output
    myPendingOutput.erase(this);
}

void
MSDevice_Tripinfo::cleanup() {
    myVehicleCount = 0;
    myTotalRouteLength = 0;
    myTotalSpeed = 0;
    myTotalDuration = 0;
    myTotalWaitingTime = 0;
    myTotalTimeLoss = 0;
    myTotalDepartDelay = 0;
    myWaitingDepartDelay = -1;

    // LFPlugin Begin
    myTotalExpectedTime = 0;
    myTotalDelayTime = 0;
    myTotalDurationNoNeg = 0;
    myTotalVehicleCount = 0;
    myTotalVehicleCountExcludeEdges = 0;

    myVehicleCountNoNeg = 0;
    myVehicleCountOnlyNeg = 0;
    myTotalTimeLossNoNeg = 0;
    myTotalTimeLossOnlyNeg = 0;
    //Made for ad-hoc log files on time-delay and tts
    /*fclose(fp_time_delay);
    fclose(fp_time_delay_excluded);
    fclose(fp_num_of_vehs);
    fclose(fp_num_of_vehs_excluded);*/
    // LFPlugin End
    myWalkCount = 0;
    myTotalWalkRouteLength = 0;
    myTotalWalkDuration = 0;
    myTotalWalkTimeLoss = 0;

    myRideCount = {0, 0};
    myRideBusCount = {0, 0};
    myRideRailCount = {0, 0};
    myRideTaxiCount = {0, 0};
    myRideBikeCount = {0, 0};
    myRideAbortCount = {0, 0};
    myTotalRideWaitingTime = {0., 0.};
    myTotalRideRouteLength = {0., 0.};
    myTotalRideDuration = {0, 0};
}

bool
MSDevice_Tripinfo::notifyIdle(SUMOTrafficObject& veh) {
    if (veh.isVehicle()) {
        myWaitingTime += DELTA_T;
        if (!myAmWaiting) {
            myWaitingCount++;
            myAmWaiting = true;
        }
    }
    return true;
}


bool
MSDevice_Tripinfo::notifyMove(SUMOTrafficObject& veh, double /*oldPos*/,
                              double /*newPos*/, double newSpeed) {
    if (veh.isStopped()) {
        myStoppingTime += DELTA_T;
    } else if (newSpeed <= SUMO_const_haltingSpeed) {
        myWaitingTime += DELTA_T;
        if (!myAmWaiting) {
            myWaitingCount++;
            myAmWaiting = true;
        }
    } else {
        myAmWaiting = false;
    }
    return true;
}


void
MSDevice_Tripinfo::notifyMoveInternal(const SUMOTrafficObject& veh,
                                      const double /* frontOnLane */,
                                      const double timeOnLane,
                                      const double /* meanSpeedFrontOnLane */,
                                      const double meanSpeedVehicleOnLane,
                                      const double /* travelledDistanceFrontOnLane */,
                                      const double /* travelledDistanceVehicleOnLane */,
                                      const double /* meanLengthOnLane */) {

    // called by meso
    const MEVehicle* mesoVeh = dynamic_cast<const MEVehicle*>(&veh);
    assert(mesoVeh);
    const double vmax = veh.getEdge()->getVehicleMaxSpeed(&veh);
    if (vmax > 0) {
        myMesoTimeLoss += TIME2STEPS(timeOnLane * (vmax - meanSpeedVehicleOnLane) / vmax);
    }
    myWaitingTime += veh.getWaitingTime();
    myStoppingTime += TIME2STEPS(mesoVeh->getCurrentStoppingTimeSeconds());
}

void
MSDevice_Tripinfo::updateParkingStopTime() {
    if (myParkingStarted >= 0) {
        myStoppingTime += (MSNet::getInstance()->getCurrentTimeStep() - myParkingStarted);
        myParkingStarted = -1;
    }
}

bool
MSDevice_Tripinfo::notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
        if (!MSGlobals::gUseMesoSim) {
            myDepartLane = static_cast<MSVehicle&>(veh).getLane()->getID();
            myDepartPosLat = static_cast<MSVehicle&>(veh).getLateralPositionOnLane();
        }
        myDepartSpeed = veh.getSpeed();
        myRouteLength = -veh.getPositionOnLane();
    } else if (reason == MSMoveReminder::NOTIFICATION_PARKING) {
        // notifyMove is not called while parking
        // @note insertion delay when resuming after parking is included
        updateParkingStopTime();
    }
    return true;
}


bool
MSDevice_Tripinfo::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/,
                               MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (reason >= MSMoveReminder::NOTIFICATION_ARRIVED) {
        myArrivalTime = MSNet::getInstance()->getCurrentTimeStep();
        myArrivalReason = reason;
        if (!MSGlobals::gUseMesoSim) {
            myArrivalLane = static_cast<MSVehicle&>(veh).getLane()->getID();
            myArrivalPosLat = static_cast<MSVehicle&>(veh).getLateralPositionOnLane();
        }
        // @note vehicle may have moved past its arrivalPos during the last step
        // due to non-zero arrivalspeed but we consider it as arrived at the desired position
        // However, vaporization may happen anywhere (via TraCI)
        if (reason > MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED) {
            // vaporized
            myArrivalPos = veh.getPositionOnLane();
        } else {
            myArrivalPos = myHolder.getArrivalPos();
        }
        myArrivalSpeed = veh.getSpeed();
        updateParkingStopTime();
    } else if (reason == MSMoveReminder::NOTIFICATION_PARKING) {
        myParkingStarted = MSNet::getInstance()->getCurrentTimeStep();
    } else if (reason == NOTIFICATION_JUNCTION || reason == NOTIFICATION_TELEPORT) {
        if (MSGlobals::gUseMesoSim) {
            myRouteLength += myHolder.getEdge()->getLength();
        } else {
            MSLane* lane = static_cast<MSVehicle&>(veh).getLane();
            if (lane != nullptr) {
                myRouteLength += lane->getLength();
            }
        }
    }
    return true;
}


void
MSDevice_Tripinfo::generateOutput(OutputDevice* tripinfoOut) const {
    const SUMOTime timeLoss = MSGlobals::gUseMesoSim ? myMesoTimeLoss : static_cast<MSVehicle&>(myHolder).getTimeLoss();

    // LFPlugin Begin
    const SUMOTime timeLossNoNeg = MSGlobals::gUseMesoSim ? myMesoTimeLoss : (static_cast<MSVehicle&>(myHolder)).getTimeLossNoNeg();
    const SUMOTime timeLossExcludeEdges = MSGlobals::gUseMesoSim ? myMesoTimeLoss : (static_cast<MSVehicle&>(myHolder)).getTimeLossExcludeEdges();
    const SUMOTime timeLossNoNegExludeEdges = MSGlobals::gUseMesoSim ? myMesoTimeLoss : (static_cast<MSVehicle&>(myHolder)).getTimeLossNoNegExcludeEdges();
    // LFPlugin End


    const double routeLength = myRouteLength + (myArrivalTime == NOT_ARRIVED ? myHolder.getPositionOnLane() : myArrivalPos);
    const SUMOTime duration = (myArrivalTime == NOT_ARRIVED ? SIMSTEP : myArrivalTime) - myHolder.getDeparture();
    //std::cout << "stats, veh:" << (static_cast<MSVehicle&>(myHolder)).getID() <<"," << routeLength << "," << " dur:" << duration << "\n";
    // LFPlugin Begin
    const SUMOTime expectedTime = ((static_cast<MSVehicle&>(myHolder)).getvehroutelength()/static_cast<MSVehicle&>(myHolder).getDesiredSpeed())*1000; // convert From sec to ms
    //const SUMOTime expectedTime = (routeLength / static_cast<MSVehicle&>(myHolder).getDesiredSpeed()) * 1000; // convert From sec to ms
    // LFPlugin End
    
    myVehicleCount++;
    myTotalRouteLength += routeLength;
    myTotalSpeed += routeLength / STEPS2TIME(duration);
    myTotalDuration += duration;
    myTotalWaitingTime += myWaitingTime;
    myTotalTimeLoss += timeLoss;
    myTotalDepartDelay += myHolder.getDepartDelay();
    
    // LFPlugin Begin
    myTotalTimeLossNoNeg += fmax(timeLoss, 0);// timeLossNoNeg;
    myVehicleCountNoNeg = (timeLoss >= 0) ? myVehicleCountNoNeg + 1 : myVehicleCountNoNeg;
    myTotalTimeLossOnlyNeg += fmin(timeLoss, 0);
    myVehicleCountOnlyNeg = (timeLoss < 0) ? myVehicleCountOnlyNeg + 1 : myVehicleCountOnlyNeg;
    myTotalTimeLossExcludeEdges += timeLossExcludeEdges;
    myTotalTimeLossNoNegExcludeEdges += fmax(timeLossExcludeEdges, 0);//timeLossNoNegExludeEdges;
    
    //Made for ad-hoc log files on time-delay and tts

    //fprintf(fp_time_delay, "%f,%f,%s\n", STEPS2TIME(timeLoss), SIMTIME, static_cast<MSVehicle&>(myHolder).getRoute().getID().c_str());
   
    
    //fflush(fp_time_delay);
    //fprintf(fp_time_delay_excluded, "%f,%f,%s\n", STEPS2TIME(timeLoss), SIMTIME, static_cast<MSVehicle&>(myHolder).getRoute().getID().c_str());
    //fflush(fp_time_delay_excluded);
    myTotalExpectedTime += expectedTime;
    const SUMOTime myDelay = duration - expectedTime;

    myTotalDelayTime = myTotalDelayTime + myDelay;
    myTotalDurationNoNeg += fmax(duration, expectedTime);
    // LFPlugin End
    myPendingOutput.erase(this);
    if (tripinfoOut == nullptr) {
        return;
    }
    // write
    OutputDevice& os = *tripinfoOut;
    os.openTag("tripinfo").writeAttr("id", myHolder.getID());
    os.writeAttr("depart", time2string(myHolder.getDeparture()));
    os.writeAttr("departLane", myDepartLane);
    os.writeAttr("departPos", myHolder.getDepartPos());
    if (MSGlobals::gLateralResolution > 0) {
        os.writeAttr("departPosLat", myDepartPosLat);
    }
    os.writeAttr("departSpeed", myDepartSpeed);
    os.writeAttr("departDelay", time2string(myHolder.getDepartDelay()));
    os.writeAttr("arrival", time2string(myArrivalTime));
    os.writeAttr("arrivalLane", myArrivalLane);
    os.writeAttr("arrivalPos", myArrivalPos);
    if (MSGlobals::gLateralResolution > 0) {
        os.writeAttr("arrivalPosLat", myArrivalPosLat);
    }
    os.writeAttr("arrivalSpeed", myArrivalSpeed);
    os.writeAttr("duration", time2string(duration));
    os.writeAttr("routeLength", routeLength);
    os.writeAttr("waitingTime", time2string(myWaitingTime));
    os.writeAttr("waitingCount", myWaitingCount);
    os.writeAttr("stopTime", time2string(myStoppingTime));
    // LFPlugin Begin
    // removed next line
    //os.writeAttr("timeLoss", time2string(timeLoss));
    // LFPlugin End
    os.writeAttr("rerouteNo", myHolder.getNumberReroutes());
    os.writeAttr("devices", toString(myHolder.getDevices()));
    os.writeAttr("vType", myHolder.getVehicleType().getID());
    os.writeAttr("speedFactor", myHolder.getChosenSpeedFactor());
    std::string vaporized;
    switch (myArrivalReason) {
        case MSMoveReminder::NOTIFICATION_VAPORIZED_CALIBRATOR:
            vaporized = "calibrator";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_GUI:
            vaporized = "gui";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_COLLISION:
            vaporized = "collision";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_VAPORIZER:
            vaporized = "vaporizer";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_TRACI:
            vaporized = "traci";
            break;
        case MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED:
            vaporized = "teleport";
            break;
        default:
            vaporized = (myHolder.getEdge() == *(myHolder.getRoute().end() - 1) ? "" : "end");

    }

    // LFPlugin Begin
    os.writeAttr("delay", time2string(timeLoss));
    // LFPlugin End

    os.writeAttr("vaporized", vaporized);
    // cannot close tag because emission device output might follow
}


void
MSDevice_Tripinfo::generateOutputForUnfinished() {
    MSNet* net = MSNet::getInstance();
    OutputDevice* tripinfoOut = (OptionsCont::getOptions().isSet("tripinfo-output") ?
                                 &OutputDevice::getDeviceByOption("tripinfo-output") : nullptr);
    myWaitingDepartDelay = 0;
    int undeparted = 0;
    int departed = 0;
    const SUMOTime t = net->getCurrentTimeStep();
    while (myPendingOutput.size() > 0) {
        const MSDevice_Tripinfo* d = *myPendingOutput.begin();
        if (d->myHolder.hasDeparted()) {
            departed++;
            const_cast<MSDevice_Tripinfo*>(d)->updateParkingStopTime();
            d->generateOutput(tripinfoOut);
            if (tripinfoOut != nullptr) {
                for (MSVehicleDevice* const dev : d->myHolder.getDevices()) {
                    if (typeid(*dev) == typeid(MSDevice_Tripinfo) || typeid(*dev) == typeid(MSDevice_Vehroutes)) {
                        // tripinfo is special and vehroute has it's own write-unfinished option
                        continue;
                    }
                    dev->generateOutput(tripinfoOut);
                }
                OutputDevice::getDeviceByOption("tripinfo-output").closeTag();
            }
        } else {
            undeparted++;
            myWaitingDepartDelay += (t - d->myHolder.getParameter().depart);
            myPendingOutput.erase(d);
        }
    }
    if (myWaitingDepartDelay > 0) {
        myWaitingDepartDelay /= undeparted;
    }
    // unfinished persons
    if (net->hasPersons()) {
        MSTransportableControl& pc = net->getPersonControl();
        while (pc.loadedBegin() != pc.loadedEnd()) {
            pc.erase(pc.loadedBegin()->second);
        }
    }

}


void
MSDevice_Tripinfo::addPedestrianData(double walkLength, SUMOTime walkDuration, SUMOTime walkTimeLoss) {
    myWalkCount++;
    myTotalWalkRouteLength += walkLength;
    myTotalWalkDuration += walkDuration;
    myTotalWalkTimeLoss += walkTimeLoss;
}


void
MSDevice_Tripinfo::addRideTransportData(const bool isPerson, const double distance, const SUMOTime duration,
                                        const SUMOVehicleClass vClass, const std::string& line, const SUMOTime waitingTime) {
    const int index = isPerson ? 0 : 1;
    myRideCount[index]++;
    if (duration > 0) {
        myTotalRideWaitingTime[index] += waitingTime;
        myTotalRideRouteLength[index] += distance;
        myTotalRideDuration[index] += duration;
        if (vClass == SVC_BICYCLE) {
            myRideBikeCount[index]++;
        } else if (!line.empty()) {
            if (isRailway(vClass)) {
                myRideRailCount[index]++;
            } else if (vClass == SVC_TAXI) {
                myRideTaxiCount[index]++;
            } else {
                // some kind of road vehicle
                myRideBusCount[index]++;
            }
        }
    } else {
        myRideAbortCount[index]++;
    }
}


std::string
MSDevice_Tripinfo::printStatistics() {
    std::ostringstream msg;
    msg.setf(msg.fixed);
    msg.precision(gPrecision);
    msg << "Statistics (avg):\n"
        << " RouteLength: " << getAvgRouteLength() << "\n"
        << " Speed: " << getAvgTripSpeed() << "\n"
        << " Duration: " << getAvgDuration() << "\n"
        << " WaitingTime: " << getAvgWaitingTime() << "\n"
        << " TimeLoss: " << getAvgTimeLoss() << "\n"
        << " DepartDelay: " << getAvgDepartDelay() << "\n";
    if (myWaitingDepartDelay >= 0) {
        msg << " DepartDelayWaiting: " << STEPS2TIME(myWaitingDepartDelay) << "\n";
    }
    if (myWalkCount > 0) {
        msg << "Pedestrian Statistics (avg of " << myWalkCount << " walks):\n"
            << " RouteLength: " << getAvgWalkRouteLength() << "\n"
            << " Duration: " << getAvgWalkDuration() << "\n"
            << " TimeLoss: " << getAvgWalkTimeLoss() << "\n";
    }
    printRideStatistics(msg, "Ride", "rides", 0);
    printRideStatistics(msg, "Transport", "transports", 1);
    return msg.str();
}

void
MSDevice_Tripinfo::printRideStatistics(std::ostringstream& msg, const std::string& category, const std::string& modeName, const int index) {
    if (myRideCount[index] > 0) {
        msg << category << " Statistics (avg of " << myRideCount[index] << " " << modeName << "):\n";
        msg << " WaitingTime: " << STEPS2TIME(myTotalRideWaitingTime[index] / myRideCount[index]) << "\n";
        msg << " RouteLength: " << myTotalRideRouteLength[index] / myRideCount[index] << "\n";
        msg << " Duration: " << STEPS2TIME(myTotalRideDuration[index] / myRideCount[index]) << "\n";
        if (myRideBusCount[index] > 0) {
            msg << " Bus: " << myRideBusCount[index] << "\n";
        }
        if (myRideRailCount[index] > 0) {
            msg << " Train: " << myRideRailCount[index] << "\n";
        }
        if (myRideTaxiCount[index] > 0) {
            msg << " Taxi: " << myRideTaxiCount[index] << "\n";
        }
        if (myRideBikeCount[index] > 0) {
            msg << " Bike: " << myRideBikeCount[index] << "\n";
        }
        if (myRideAbortCount[index] > 0) {
            msg << " Aborted: " << myRideAbortCount[index] << "\n";
        }
    }

}


void
MSDevice_Tripinfo::writeStatistics(OutputDevice& od) {
    od.setPrecision(gPrecision);
    od.openTag("vehicleTripStatistics");
    od.writeAttr("routeLengthAvg", getAvgRouteLength());
    od.writeAttr("speedAvg", getAvgTripSpeed());
    od.writeAttr("durationAvg", getAvgDuration());
    od.writeAttr("waitingTimeAvg", getAvgWaitingTime());
    // LFPlugin Begin
    // removed next line
    //od.writeAttr("delayInitial", getAvgDelay());
    //od.writeAttr("delayInitialNoNeg", getAvgDelayNoNeg());
    // LFPlugin End
    od.writeAttr("departDelayAvg", getAvgDepartDelay());
    od.writeAttr("departDelayWaiting", myWaitingDepartDelay);
    // LFPlugin Begin
    od.writeAttr("delayAvg", getAvgTimeLoss());
    od.writeAttr("delayAvgNoNeg", getAvgTimeLossNoNeg());
    od.writeAttr("delayAvgOnlyNeg", getAvgTimeLossOnlyNeg());
    od.writeAttr("totalTimeSpent_hours", getTotalTimeSpent());
    if(OptionsCont::getOptions().isSet("exclude-edges-from-metrics")){
        od.writeAttr("delayAvg_ExcludeEdges", getAvgTimeLossExcludeEdges());
        od.writeAttr("delayAvgNoNeg_ExcludeEdges", getAvgTimeLossNoNegExcludeEdges());
        od.writeAttr("totalTimeSpent_hours_ExcludeEdges", getTotalTimeSpentwithExcluded());
    }
    // od.writeAttr("totalTimeSpent_sec", getTotalTimeSpent() * 3600);
    // od.writeAttr("totalDuration", myTotalDuration);
    // od.writeAttr("total delay", time2string(myTotalDelayTime));
    // od.writeAttr("Avg delay", time2string(myAvgDelay));
    // od.writeAttr("total expected time", time2string(myTotalExpectedTime));
    // LFPlugin End
    od.closeTag();
    od.openTag("pedestrianStatistics");
    od.writeAttr("number", myWalkCount);
    od.writeAttr("routeLengthAvg", getAvgWalkRouteLength());
    od.writeAttr("durationAvg", getAvgWalkDuration());
    od.writeAttr("timeLossAvg", getAvgWalkTimeLoss());
    od.closeTag();
    writeRideStatistics(od, "rideStatistics", 0);
    writeRideStatistics(od, "transportStatistics", 1);
}

void
MSDevice_Tripinfo::writeRideStatistics(OutputDevice& od, const std::string& category, const int index) {
    od.openTag(category);
    od.writeAttr("number", myRideCount[index]);
    if (myRideCount[index] > 0) {
        od.writeAttr("waitingTime", STEPS2TIME(myTotalRideWaitingTime[index] / myRideCount[index]));
        od.writeAttr("routeLength", myTotalRideRouteLength[index] / myRideCount[index]);
        od.writeAttr("duration", STEPS2TIME(myTotalRideDuration[index] / myRideCount[index]));
        od.writeAttr("bus", myRideBusCount[index]);
        od.writeAttr("train", myRideRailCount[index]);
        od.writeAttr("taxi", myRideTaxiCount[index]);
        od.writeAttr("bike", myRideBikeCount[index]);
        od.writeAttr("aborted", myRideAbortCount[index]);
    }
    od.closeTag();
}

// LFPlugin Begin
double
MSDevice_Tripinfo::getTotalTimeSpent() {
    return myTotalVehicleCount * TS / 3600;
}

double
MSDevice_Tripinfo::getTotalTimeSpentwithExcluded() {
    return myTotalVehicleCountExcludeEdges * TS / 3600;
}
// LFPlugin End


double
MSDevice_Tripinfo::getAvgRouteLength() {
    if (myVehicleCount > 0) {
        return myTotalRouteLength / myVehicleCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgTripSpeed() {
    if (myVehicleCount > 0) {
        return myTotalSpeed / myVehicleCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgDuration() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalDuration / myVehicleCount);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgWaitingTime() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalWaitingTime / myVehicleCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgTimeLoss() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalTimeLoss / myVehicleCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgDepartDelay() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalDepartDelay / myVehicleCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgWalkRouteLength() {
    if (myWalkCount > 0) {
        return myTotalWalkRouteLength / myWalkCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgWalkDuration() {
    if (myWalkCount > 0) {
        return STEPS2TIME(myTotalWalkDuration / myWalkCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgWalkTimeLoss() {
    if (myWalkCount > 0) {
        return STEPS2TIME(myTotalWalkTimeLoss / myWalkCount);
    } else {
        return 0;
    }
}

// LFPlugin Begin
double
MSDevice_Tripinfo::getAvgTimeLossNoNeg() {
    if (myVehicleCountNoNeg > 0) {
        return STEPS2TIME(myTotalTimeLossNoNeg / myVehicleCountNoNeg);
    }
    else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgTimeLossOnlyNeg() {
    if (myVehicleCountOnlyNeg > 0) {
        return STEPS2TIME(myTotalTimeLossOnlyNeg / myVehicleCountOnlyNeg);
    }
    else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgTimeLossExcludeEdges() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalTimeLossExcludeEdges / myVehicleCount);
    }
    else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgTimeLossNoNegExcludeEdges() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalTimeLossNoNegExcludeEdges / myVehicleCount);
    }
    else {
        return 0;
    }
}

double 
MSDevice_Tripinfo::getAvgDelay() {
    if (myVehicleCount > 0) {
        return STEPS2TIME((myTotalDuration - myTotalExpectedTime)/myVehicleCount);
    }
    else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgDelayNoNeg() {
    if (myVehicleCount > 0) {
        return STEPS2TIME((myTotalDurationNoNeg - myTotalExpectedTime) / myVehicleCount);
    }
    else {
        return 0;
    }
}
// LFPlugin End

double
MSDevice_Tripinfo::getAvgRideDuration() {
    if (myRideCount[0] > 0) {
        return STEPS2TIME(myTotalRideDuration[0] / myRideCount[0]);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgRideWaitingTime() {
    if (myRideCount[0] > 0) {
        return STEPS2TIME(myTotalRideWaitingTime[0] / myRideCount[0]);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgRideRouteLength() {
    if (myRideCount[0] > 0) {
        return myTotalRideRouteLength[0] / myRideCount[0];
    } else {
        return 0;
    }
}


void
MSDevice_Tripinfo::saveState(OutputDevice& out) const {
    out.openTag(SUMO_TAG_DEVICE);
    out.writeAttr(SUMO_ATTR_ID, getID());
    std::ostringstream internals;
    if (!MSGlobals::gUseMesoSim) {
        internals << myDepartLane << " " << myDepartPosLat << " ";
    }
    internals << myDepartSpeed << " " << myRouteLength << " " << myWaitingTime << " " << myAmWaiting << " " << myWaitingCount << " ";
    internals << myStoppingTime << " " << myParkingStarted;
    out.writeAttr(SUMO_ATTR_STATE, internals.str());
    out.closeTag();
}


void
MSDevice_Tripinfo::loadState(const SUMOSAXAttributes& attrs) {
    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    if (!MSGlobals::gUseMesoSim) {
        bis >> myDepartLane >> myDepartPosLat;
    }
    bis >> myDepartSpeed >> myRouteLength >> myWaitingTime >> myAmWaiting >> myWaitingCount;
    bis >> myStoppingTime >> myParkingStarted;
}


/****************************************************************************/
