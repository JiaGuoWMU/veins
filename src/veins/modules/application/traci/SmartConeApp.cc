/*
 * SmartConeApp.cc
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
        //Initializing members and pointers of your application goes here
        suggestedSpeed = 40; // based on LOS
        minSpeed = 53.3; // based on LOS
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

void SmartConeApp::onBSM(BasicSafetyMessage* bsm) {
    // The application has received a beacon message from another car or RSU
    // code for handling the message goes here

}

void SmartConeApp::onWSM(WaveShortMessage* wsm) {
    // The application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    // Receive a message with the target laneId and target speed,
    // slow down to that speed and change to the left lane (construction at right lane)

    // split the message to get laneId and speed
    char *msg = (char *)wsm->getWsmData();
    char *s = strtok(msg, " ");
    std::string targetLaneId(s);
    float speed = atof(strtok(NULL, " "));

    // check if the vehicle is on the target lane
    if (traciVehicle->getLaneId() == targetLaneId) {
        //TODO: the duration should be at least the time estimated to pass the construction zone
        // the duration is set to the max simulation time.
        traciVehicle->changeLane(1, 1000 * 4000); // merge to the left lane
        /*
         * The enumeration index of the lane (0 is the rightmost lane, <NUMBER_LANES>-1 is the leftmost one)
         */
        traciVehicle->slowDown(speed, 5000); //slow down over for at least 5s
    } else {
        // ignore the message
    }

    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }

}


void SmartConeApp::onWSA(WaveServiceAdvertisment* wsa) {
    // The application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples

}

void SmartConeApp::handleSelfMsg(cMessage* msg) {
    // this method is for self messages (mostly timers)
    // it is important to call the BaseWaveApplLayer function for BSM and WSM transmission
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
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    BaseWaveApplLayer::handlePositionUpdate(obj);


    // Since all the probe vehicle are doing this at the same time, we do not
    // need to explicitly check all the edges

    if (traci->getCurrentTime() % 3000 == 0) { // update every 3000ms
        std::string roadId = traciVehicle->getRoadId(); // the id of the edge the car is at
        double meanSpeed = traci->road(roadId).getMeanSpeed(); // the mean speed of the edge
        if (meanSpeed < minSpeed && roadId != "1") { // congested and not the first road
            std::stringstream msg;
            msg << atoi(roadId.c_str()) - 1 << "_0 " << suggestedSpeed; // right lane of the prev edge
            std::string message = msg.str();
            sendMessage(message);
            lastSent = simTime();
        } else {
            // ignore
        }
    }
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

void SmartConeApp::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}
