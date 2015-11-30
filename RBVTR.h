/*
 * RBVTRPacket.h
 *
 *  Created on: Jul 6, 2015
 *      Author: chengli
 */

#ifndef __INET_RBVTR_H_
#define __INET_RBVTR_H_

#include <cmessage.h>
#include <RouteInterface.h>
#include <RBVTR/RBVTRPacket.h>
#include "IPSocket.h"
//#include "GPSR_m.h"
#include"RouteInterface/SeenTable.h"
#include "RoadTable.h"
#include "RouteInterface/BroadcastWaitingTable.h"
#include "RouteInterface/DelayPacketTable.h"
#include "RTSPacketTable.h"


#include <algorithm>

class INET_API RBVTR:   public RouteInterface {
public:
    RBVTR();
    virtual ~RBVTR();
    typedef std::map<IPvXAddress, int> RDPacketSeenlist;

protected:

    double HoldingIndex ;
    double recPow;
    simtime_t RDliftime;
    simtime_t RRliftime;
    simtime_t RUliftime;
    simtime_t DATAliftime;
    simtime_t nextRUtimer;
    cMessage * RUTimer;

    double Tmax;
    double dopt;
    double dmax;

    double a1;
    double a2;
    double a3;

    string oldroadID;
    SeenTable RDSeenlist;
    SeenTable RSTwatinglist;
    SeenTable RUSeenlist;
    SeenTable DATASeenlist;

    std::vector<string> RSTSeenlist;

    RoadTable routingRoad;
     RTSPacketTable RTSlist;
    IInterfaceTable *interfaceTable;

     void processSelfMessage(cMessage * message);
     void processMessage(cPacket * ctrlPacket,IPv4ControlInfo *udpProtocolCtrlInfo);
     void  processRUTimer(simtime_t timer);

     void initialize(int stage);
     void   test(RBVTRPacket* content);
     RBVTRPacket *createRDPacket(IPvXAddress destination,  std::string packetname);
     RBVTRPacket *createRRPacket(RBVTRPacket *rbvtrPacket);
     RBVTRPacket *createDataPacket(IPvXAddress destination, cPacket * content);
     RBVTRPacket *createRUPacket(IPvXAddress destination,  std::vector<std::string>roads);
     RBVTRPacket *createRTSPacket(RBVTRPacket *rbvtrPacket);
     RBVTRPacket *createCTSPacket(RBVTRPacket *rbvtrPacket);

     void  processRTSPACKET(RBVTRPacket * rbvtrPacket);
     void  processCTSPACKET(RBVTRPacket * rbvtrPacket);
     void  processRDPACKET(RBVTRPacket * rbvtrPacket);
     void  processRUPACKET(RBVTRPacket * rbvtrPacket);
     void  processRRPACKET(RBVTRPacket * rbvtrPacket);

     void  BroadcastRTS(RBVTRPacket * rbvtrPacket);

   // void  handleMessage(cMessage *message);
    simtime_t  CaculateHoldTime(Coord srcPosition,Coord desPosition);
    simtime_t CaculateHoldTime(Coord srcPosition);

    double   CaculateF(double distence);
    double   CaculateP(double distence);
    double  gaussrand(double o);

    void  scheduleReBoardcastRDTimer(simtime_t holdingtime,RBVTRPacket *rbvtrPacket,IPv4Datagram * datagram);

    //typedef std::map<cMessage *,IPvXAddress> AddressToSqum;
    PacketWaitingTable packetwaitinglist;
   // std::vector< > reBoardcastRDTimerlist ;
    void  processRDTimer(cMessage * message,RBVTRPacket *rbvtrPacket,const IPv4Datagram * pakcet);

    void  clearMessage(cMessage * message,RBVTRPacket *rbvtrPacket);
    void sendDataPacket(const IPvXAddress& target,std::vector<std::string> roads,const IPvXAddress nexthop);
    virtual void receiveChangeNotification(int category, const cObject *details);

private:

    virtual Result datagramPreRoutingHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop);
    virtual Result datagramForwardHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop);
    virtual Result datagramPostRoutingHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop) ;
    virtual Result datagramLocalInHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry);
    virtual Result datagramLocalOutHook(IPv4Datagram * datagram, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop);//{ return ACCEPT; }

    int squmRD;
    int squmRR;
    int squmRU;
    int squmRTS;
    int squmDATA;

};

#endif /* RBVTRPACKET_H_ */