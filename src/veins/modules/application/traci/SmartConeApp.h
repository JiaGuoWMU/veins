/*
 * SmartConeApp.h
 * Author: Jia Guo
 * Created on: 03/31/2018
 */

#ifndef SmartConeApp_H
#define SmartConeApp_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

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
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		std::string lastRoad;

		double suggestedSpeed;
		double minSpeed;
		int arrived;
        simtime_t lastSent; // the last time this sent a message

	protected:
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void handlePositionUpdate(cObject* obj);
//        virtual void handleSelfMsg(cMessage* msg);
//        virtual void onWSA(WaveServiceAdvertisment* wsa);
};

#endif
