/*
 * SmartConeApp.cc
 * Author: Jia Guo
 * Created on: 03/31/2018
 */

#include "veins/modules/application/traci/SmartConeApp.h"
#include <stdlib.h>
#include <sstream>
#include <cstring>

Define_Module(SmartConeApp);

void SmartConeApp::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        currentSubscribedServiceId = -1;
        //Initializing members and pointers of your application goes here
        suggestedSpeed = 17.88; // m/s based on LOS
        minSpeed = 23.83; // m/s based on LOS
        arrived = 0;
        //setup veins pointers
        lastSent = simTime();
        traciVehicle->setLaneChangeMode(512);
        /*
         * The default lane change mode is 0b011001010101 = 1621
         * which means that the laneChangeModel may execute all changes unless in conflict with TraCI.
         * Requests from TraCI are handled urgently but with full consideration for safety constraints.
         * To disable all autonomous changing but still handle safety checks in the simulation, either one of the modes 256 (collision avoidance)
         * or 512 (collision avoidance and safety-gap enforcement) may be used.
         */
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
    // The application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    // Receive a message with the target laneId and target speed,
    // slow down to that speed and change to the left lane (construction at right lane)

    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }
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
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    BaseWaveApplLayer::handlePositionUpdate(obj);
    if (!sentMessage && traciVehicle->getLaneId() == mobility->getManager()->getRoadOfInterest()) {
        sentMessage = true;
        WaveShortMessage* wsm = new WaveShortMessage();
        populateWSM(wsm);
        wsm->setWsmData(mobility->getManager()->getRoadOfInterest().c_str());
        //host is standing still due to crash
        if (dataOnSch) {
            startService(Channels::SCH2, 42, "Traffic Information Service");
            //started service and server advertising, schedule message to self to send later
            scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
        }
        else {
            //send right away on CCH, because channel switching is disabled
            sendDown(wsm);
        }

        //std::cout << "********Lane change now" << endl;
        //TODO: the duration should be at least the time estimated to pass the construction zone
        // the duration is set to the max simulation time.
        traciVehicle->changeLane(1, 1000 * 4000); // merge to the left lane
        /*
         * The enumeration index of the lane (0 is the rightmost lane, <NUMBER_LANES>-1 is the leftmost one)
         */
        traciVehicle->slowDown(suggestedSpeed, 5000); //slow down over for at least 5s
    } else {
        // no action taken
    }
}
