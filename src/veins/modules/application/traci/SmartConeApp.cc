/*
 * SmartConeAppl.cpp
 * Author: Jia Guo
 * Created on: 03/31/2018
 */

#include "veins/modules/application/traci/SmartConeApp.h"
#include <stdlib.h>
#include <sstream>
#include <cstring>

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;


Define_Module(SmartConeApp);

void SmartConeApp::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //setup veins pointers
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        lastSent = simTime();
        traciVehicle->setLaneChangeMode(512);
        /*
         * The default lane change mode is 0b011001010101 = 1621
         * which means that the laneChangeModel may execute all changes unless in conflict with TraCI.
         * Requests from TraCI are handled urgently but with full consideration for safety constraints.
         * To disable all autonomous changing but still handle safety checks in the simulation, either one of the modes 256 (collision avoidance)
         * or 512 (collision avoidance and safety-gap enforcement) may be used.
         */

        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void SmartConeApp::onWSA(WaveServiceAdvertisment* wsa) {
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void SmartConeApp::onWSM(WaveShortMessage* wsm) {
    // Receive a message with the target laneId and target speed, slow down to that speed and change lane
    // std::string message = wsm->getWsmData();

    // split the message to get laneId and speed
    char *msg = (char *)wsm->getWsmData();
    char *s = strtok(msg, " ");
    std::string targetLaneId(s);
    float speed = atof(strtok(NULL, " "));

    // check if the vehicle is on the target lane
    if (traciVehicle->getLaneId() == targetLaneId) {
        traciVehicle->slowDown(speed, 5000); //slow down over 1s
        //TODO: the duration should be at least the time estimated to pass the construction zone
        // the duration is set to the max simulation time now.
        traciVehicle->changeLane(1, 1000 * 4000); // merge to the left lane
        /*
         * The enumeration index of the lane (0 is the rightmost lane, <NUMBER_LANES>-1 is the leftmost one)
         */
    } else {
        // ignore the message
    }

//    findHost()->getDisplayString().updateWith("r=16,green");
//
//    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
//    if (!sentMessage) {
//        sentMessage = true;
//        //repeat the received traffic update once in 2 seconds plus some random delay
//        wsm->setSenderAddress(myId);
//        wsm->setSerial(3);
//        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
//    }
}

void SmartConeApp::handleSelfMsg(cMessage* msg) {
    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
        //send this message on the service channel until the counter is 3 or higher.
        //this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 3) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
            scheduleAt(simTime()+1, wsm);
        }
    }
    else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void SmartConeApp::handlePositionUpdate(cObject* obj) {

    BaseWaveApplLayer::handlePositionUpdate(obj);
    double suggestedSpeed = 40; // based on LOS
    double minSpeed = 53.3; // based on LOS

    // TODO: update every 30s
    // Actually, all the probe vehicle are doing this at the same time, so we may
    // not need to explicitly check all the edges
//    if (simTime() % 30 == 0) {
//
//    }
    for (int i = 2; i <= 10; i++) {
        std::stringstream id;
        id << i << "_0";
        std::string laneId = id.str();
        double meanSpeed = traci->lane(laneId).getMeanSpeed();
        if (meanSpeed < minSpeed) {
            std::stringstream msg;
            msg << laneId << " " << suggestedSpeed;
            std::string message = msg.str();
            sendMessage(message);
            lastSent = simTime();
            break;
        } else {
            continue;
        }
    }

    //sends message every 5 seconds
//    if (simTime() - lastSent >= 5) {
//        std::string message = std::to_string(suggestedSpeed);
//        sendMessage(message);
//        lastSent = simTime();
//    }

//    BaseWaveApplLayer::handlePositionUpdate(obj);
//
//    // stopped for for at least 10s?
//    if (mobility->getSpeed() < 1) {
//        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {
//            findHost()->getDisplayString().updateWith("r=16,red");
//            sentMessage = true;
//
//            WaveShortMessage* wsm = new WaveShortMessage();
//            populateWSM(wsm);
//            wsm->setWsmData(mobility->getRoadId().c_str());
//
//            //host is standing still due to crash
//            if (dataOnSch) {
//                startService(Channels::SCH2, 42, "Traffic Information Service");
//                //started service and server advertising, schedule message to self to send later
//                scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
//            }
//            else {
//                //send right away on CCH, because channel switching is disabled
//                sendDown(wsm);
//            }
//        }
//    }
//    else {
//        lastDroveAt = simTime();
//    }
}


void SmartConeApp::sendMessage(std::string msg) {
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm);
    wsm->setWsmData(msg.c_str());

    if (dataOnSch) {
        startService(Channels::SCH2, 42, "Traffic Information Service");
        //started service and server advertising, schedule message to self to send later
        scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
    }
    else {
        //send right away on CCH, because channel switching is disabled
        sendDown(wsm);
    }
}
