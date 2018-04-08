/*
 * SmartConeApp.h
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


/**
 * @brief
 * The smartcone application. When the current edge/road is under the expected LOS,
 * it will send a message out to other cars on the construction lane of the previous
 * edge/road. Receiving cars will then trigger a slow down and change lane via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a WAVE Service Advertisement
 * on the CCH.
 *
 * @author Jia Guo
 *
 */

class SmartConeApp : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void finish();
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		double suggestedSpeed;
		double minSpeed;

		TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;
        simtime_t lastSent; // the last time this sent a message

	protected:
        virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
		virtual void handlePositionUpdate(cObject* obj);

		void sendMessage(std::string blockedRoadId);
};

#endif
