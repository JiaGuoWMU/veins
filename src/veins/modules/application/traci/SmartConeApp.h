/*
 * SmartConeAppl.h
 * Author: Jia Guo
 * Created on: 03/31/2018
 */

#ifndef SmartConeApp_H
#define SmartConeApp_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
//using Veins::AnnotationManager; //add for annotations


/**
 * @brief
 * A tutorial demo for TraCI. When the car is stopped for longer than 10 seconds
 * it will send a message out to other cars containing the blocked road id.
 * Receiving cars will then trigger a reroute via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a WAVE Service Advertisement
 * on the CCH.
 *
 * @author Christoph Sommer : initial DemoApp
 * @author David Eckhoff : rewriting, moving functionality to BaseWaveApplLayer, adding WSA
 *
 */

class SmartConeApp : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;

		TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;
        simtime_t lastSent; // the last time this sent a message

	protected:
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
		virtual void handlePositionUpdate(cObject* obj);

		void sendMessage(std::string blockedRoadId);
};

#endif
