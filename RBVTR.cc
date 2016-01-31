/*
 * RBVTRPacket.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: chengli
 */

#include <RBVTR/RBVTR.h>
#define RBVTR_EV EV << "RBVTR at " << getHostName() << " "
#include "TraCIMobility.h"
#include "GPSR_m.h"
#include "Radio80211aControlInfo_m.h"

Define_Module(RBVTR);

RBVTR::RBVTR(){
    interfaceTable=NULL;
}

RBVTR::~RBVTR(){

}

void RBVTR::initialize( int stage){
    RouteInterface::initialize(stage);

    if (stage == 0)
    {
        std::string myroad=getRoadID();
        EV_LOG("RBVTR",getHostName()+"road "+myroad);
        IPSocket socket(gate("ipOut"));
        socket.registerProtocol(IP_PROT_MANET);
        squmRD=0;
        squmRR=0;
        squmRU=0;
        squmDATA=0;
        squmRTS=0;

        HoldingIndex = par("HoldingIndex");

        RDliftime= par("RDliftime");
        RRliftime= par("RRliftime");
        DATAliftime= par("DATAliftime");
        nextRUtimer= par("nextRUtimer");
        RUliftime= par("RUliftime");
        routingTable = check_and_cast<IRoutingTable *>(getModuleByPath(par("routingTableModule")));
        RUTimer = new cMessage("RUTimer");
          Tmax= par("Tmax");;
          dopt= par("dopt");;
          dmax= par("dmax");;

          a1= par("a1");;
          a2= par("a2");;
          a3= par("a3");;
        //reBoardcastRDTimer = new cMessage("ReBoardcastRDTimer");;

    }else{
    if (stage == 4)
        {
        oldroadID=getRoadID();
        RouteInterface::configureInterfaces(par("interfaces"));
        EV_LOG("RBVTR",getHostName());
        nb->subscribe(this, NF_LINK_FULL_PROMISCUOUS);
        scheduleAt(simTime() + nextRUtimer, RUTimer);
        }
    }
    //: public cSimpleModule, public ILifecycle, public INotifiable, public INetfilter::IHookRouteInterface::initialize();
 }
simtime_t RBVTR::CaculateHoldTime(Coord srcPosition)
{
    return 1/(srcPosition - getSelfPosition()).length()*2;
}
simtime_t RBVTR::CaculateHoldTime(Coord srcPosition,Coord desPosition)
{
    double distence=0;
     if (desPosition==Coord(0,0,0))
     {
         distence =  (srcPosition - getSelfPosition()).length();

     }else
     {
        distence = (srcPosition-desPosition).length()-(desPosition - getSelfPosition()).length();
        if(distence<0)
            distence=dmax;
     }
    double parmax=(pow(dmax,a1)*pow(CaculateF(dopt),a2)*pow(0.2,a3));
    double A=-Tmax/parmax;
    double f=CaculateF(distence);
    double p=recPow;
    RBVTR_EV<<"A  "<<A<<endl;
    double ss= pow(distence,a1)*pow(f,a2)*pow(p,a3);
    RBVTR_EV<<" f "<<f <<" p "<<p<<" ss "<<ss<<" parmax  "<<parmax<<endl;

   // simtime_t result=A*pow(distence,a1)*pow(f,a2)*pow(p,a3);//+Tmax;
   // RBVTR_EV<<"wait "<<result<<"  seconds with distence "<<distence<<endl;
    simtime_t result=A*pow(distence,a1)*pow(f,a2)*pow(p,a3)+Tmax;
    RBVTR_EV<<"wait "<<result<<"  seconds with distence "<<distence<<endl;

    return result;
 }
double  RBVTR::CaculateF(double distence)
{
    double dtrans=dmax-dopt;
    RBVTR_EV<<" distence "<<distence<<endl;
   if(dmax<distence)
       distence=dmax-1;
   if(distence<=dopt)
   {
       return distence+dtrans;
   }
   else
   {
       return dmax- distence;
   }
}
double  RBVTR::CaculateP(double distence)
{
    double B=3.25;
    double d0=100;
    double o=4.0;
    return -10*B*log(distence/d0)+gaussrand(o);
}

double RBVTR::gaussrand(double o)
{
 double V1, V2, S;
 o=o*o;
int phase = 0;
double X;
if ( phase == 0 ) {
    do {
     double U1 = (double)rand() / RAND_MAX;
     double U2 = (double)rand() / RAND_MAX;
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
        X = V1 * sqrt(-2 * log(S) / S);
    } else
        X = V2 * sqrt(-2 * log(S) / S);

    phase = 1 - phase;

    return X*o;
}
void RBVTR::scheduleReBoardcastRDTimer(simtime_t holdingtime,RBVTRPacket *rbvtrPacket,IPv4Datagram * datagram)
{
    EV_LOG("RBVTR", "Scheduling ReBoardcast timer" );
    std::string timername="reBoardcastRDTimer_";
    if (rbvtrPacket->getPacketType() ==RBVTR_RR)
    {
     timername="reBoardcastRRTimer_";
    }
    else
    {
        if (rbvtrPacket->getPacketType() ==RBVTR_DATA)
            {
            timername="reBoardcastDATATimer_";
            }
    }
    timername=timername+rbvtrPacket->getName();
    EV_LOG("RBVTR", timername);
    cMessage * reBoardcastRDTimer = new cMessage(timername.c_str());
    packetwaitinglist.addPacket(reBoardcastRDTimer,rbvtrPacket,datagram);
    scheduleAt(simTime() + holdingtime, reBoardcastRDTimer);
}

void RBVTR::processRUTimer(simtime_t timer)
{
    if(oldroadID!=getRoadID())
    {
       std::vector<IPvXAddress> IPs= routingRoad.getAllIP();
       for(int i=0;i<IPs.size();i++)
       {
          // std::vector<std::string>roads=routingRoad.getRoadTable(IPs[i]);
           routingRoad.findnewRoadid(IPs[i],getRoadID());
           RBVTRPacket *rbvtrPacket=createRUPacket(  IPs[i],routingRoad.getRoadTable(IPs[i]));
           RUSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
           sendRIPacket(rbvtrPacket,rbvtrPacket->getdesAddress(),255,0);
       }
       oldroadID==getRoadID();
    }
    EV_LOG("RBVTR", "Scheduling RU timer" );
    scheduleAt(simTime() + timer, RUTimer);
 }
void RBVTR::processSelfMessage(cMessage * message)
{
    if (message == RUTimer)
        processRUTimer(nextRUtimer);
    else{
        RBVTRPacket* mypacket=NULL;
        mypacket =check_and_cast<RBVTRPacket *>(packetwaitinglist.getcPacket(message));
       if (mypacket!=NULL)
       {
           processRDTimer(message,mypacket,packetwaitinglist.getDataPacket(message));
           clearMessage(message,mypacket);
       }
    }
}

void RBVTR::clearMessage(cMessage * message,RBVTRPacket *rbvtrPacket)
{
     //networkProtocol->dropQueuedDatagram(dynamic_cast<const IPv4Datagram *>(rbvtrPacket));

     packetwaitinglist.removePacket(message);
     cancelAndDelete(message);
     //cancelEvent(message);
     networkProtocol->dropQueuedDatagram(dynamic_cast<const IPv4Datagram *>(rbvtrPacket));
 }

void RBVTR::processRDTimer(cMessage * message,RBVTRPacket *rbvtrPacket,const IPv4Datagram * pakcet)
{
    EV_LOG("RBVTR", "processRDTimer" );
    if(rbvtrPacket->getPacketType()!=RBVTR_DATA)
    {
        MulticastRIPacket(rbvtrPacket);
    }else
    {
        if(pakcet!=NULL)
        {
       networkProtocol->reinjectQueuedDatagram( const_cast<const IPv4Datagram *>(pakcet));
        }else
        {
            EV_LOG("RBVTR","I got a NULL");
        }
    }
}
void RBVTR::BroadcastRTS(RBVTRPacket * rbvtrPacket)
{
   RBVTRPacket *RTSPacket=createRTSPacket(rbvtrPacket);
   std::string name=RTSPacket->getName();
   RSTSeenlist.push_back(name);
   MulticastRIPacket(RTSPacket);
   //RTSlist.addPacket(RTSPacket->getSeqnum(),rbvtrPacket,NULL);
}

void RBVTR::processRDPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
         {
             EV_LOG("RBVTR","I got a RD");
             std::vector <std::string> routingroad;
             for(int i=0;i<rbvtrPacket->getroads().size();i++)
             {
                  routingroad.push_back( rbvtrPacket->getroads()[rbvtrPacket->getroads().size()-1-i]);
              }

             if(routingRoad.addRoadTable(rbvtrPacket->getsrcAddress(),routingroad,rbvtrPacket->getscrPosition()))
             {
             //reply RR
                  RBVTRPacket *RrPacket=createRRPacket(  rbvtrPacket);
               //  MulticastRIPacket(RrPacket);
                 // BroadcastRTS(RrPacket);
                 sendRIPacket(RrPacket,RrPacket->getdesAddress(),255,0);
             }
           }
         else
         {
             if(!RDSeenlist.isSeenPacket(rbvtrPacket->getsrcAddress(),rbvtrPacket->getSeqnum()))
             {
                 RBVTR_EV<<"no seen RDpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
                 std::vector<std::string> routingroads=rbvtrPacket->getroads();
                 for (int i=0 ;i<routingroads.size();i++)
                 {
                     RBVTR_EV<<"road in packet: "<<routingroads[i]<<endl;
                 }
                 if(routingroads.size()==0||routingroads[routingroads.size()-1]!=getRoadID())
                 {
                     if(routingroads.size()>=2&&hasIntersection(getRoadID(),getRoadIntersection(routingroads[routingroads.size()-2],routingroads[routingroads.size()-1])))
                     {
                         RBVTR_EV<<"Delete intersection"<<endl;
                         rbvtrPacket->getroads()[routingroads.size()-1]=getRoadID();
                     }else
                     {
                     rbvtrPacket->addroad(getRoadID());
                     }
                 }
                 RBVTR_EV<<"My road is:  "<<getRoadID()<<endl;
               routingroads=rbvtrPacket->getroads();
               for (int i=0 ;i<routingroads.size();i++)
               {
                   RBVTR_EV<<"new road in packet: "<<routingroads[i]<<endl;
               }
                // scheduleReBoardcastRDTimer(CaculateHoldTime(rbvtrPacket->getscrPosition(),rbvtrPacket->getdesPosition()),rbvtrPacket,NULL);
               scheduleReBoardcastRDTimer(CaculateHoldTime(rbvtrPacket->getscrPosition()),rbvtrPacket,NULL);

               RDSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
             }
             else
             {
                 RBVTR_EV<<"seen a RDpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
                 cMessage *mymsg=NULL;
                 mymsg=packetwaitinglist.findPacket(rbvtrPacket);
                 if(mymsg!=NULL)
                   clearMessage(mymsg,rbvtrPacket);
             }
         }
}
void RBVTR::processRRPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
         {
            EV_LOG("RBVTR","I got a RR");
            RBVTR_EV <<(rbvtrPacket->getdesAddress())<<"           "<<getSelfIPAddress()<<endl;
            routingRoad.addRoadTable(rbvtrPacket->getsrcAddress(),rbvtrPacket->getroads(),rbvtrPacket->getscrPosition());
            RBVTR_EV << "Add raod in table :"<<rbvtrPacket->getsrcAddress()<<endl;
           // BroadcastRTS(rbvtrPacket);

           //  sendDataPacket((rbvtrPacket->getsrcAddress()),rbvtrPacket->getroads());
           // RRSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());

           }
         else
         {
             EV_LOG("RBVTR","ERROR:Others' RR");
         }
}
void RBVTR::processRUPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
    {
         if(routingRoad.hasRoadTable(rbvtrPacket->getsrcAddress())){
             std::vector <std::string> routingroad;
        for(int i=0;i<rbvtrPacket->getroads().size();i++)
        {
             routingroad.push_back( rbvtrPacket->getroads()[rbvtrPacket->getroads().size()-1-i]);
         }
             routingRoad.updataRoadTable(rbvtrPacket->getsrcAddress(),routingroad,rbvtrPacket->getscrPosition());
         }
         // sendRIPacket(rbvtrPacket, IPvXAddress::LOOPBACK_ADDRESS,255,0);
       }
           else
       {
          std::vector <std::string> routingroad=rbvtrPacket->getroads();
          if(std::find(routingroad.begin(),routingroad.end(),getRoadID())!=routingroad.end())
          {
              if(!RUSeenlist.isSeenPacket(rbvtrPacket->getsrcAddress(),rbvtrPacket->getSeqnum()))
                  {
                  RBVTR_EV<<"no seen RUpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;

                   scheduleReBoardcastRDTimer(CaculateHoldTime(rbvtrPacket->getscrPosition(),rbvtrPacket->getdesPosition()),rbvtrPacket,NULL);
                   RUSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
                   }
              else
              {
                  RBVTR_EV<<"a seen RUpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;

                  cMessage *mymsg=NULL;
                  mymsg=packetwaitinglist.findPacket(rbvtrPacket);
                   if(mymsg!=NULL)
                    clearMessage(mymsg,rbvtrPacket);
              }
           }
      }
}
void RBVTR::processRTSPACKET(RBVTRPacket * rbvtrPacket)
{
   // if(rbvtrPacket->getsrcAddress()!=getSelfIPAddress())
   // {
          std::vector <std::string> routingroad=rbvtrPacket->getroads();
          std::string name=rbvtrPacket->getName();
          RBVTR_EV<<"RTS:"<<name<<endl;

          for(int i=0;i<RSTSeenlist.size();i++)
          {
              RBVTR_EV<<"RTSSEEN:"<<RSTSeenlist[i]<<endl;
          }
          if((std::find(routingroad.begin(),routingroad.end(),getRoadID())!=routingroad.end())&&(std::find(RSTSeenlist.begin(),RSTSeenlist.end(),(name))==RSTSeenlist.end()))
          {
                RBVTR_EV<<"got RTS IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
                RBVTRPacket *ctspacket=  createCTSPacket( rbvtrPacket);
                scheduleReBoardcastRDTimer(CaculateHoldTime(ctspacket->getscrPosition(),ctspacket->getdesPosition()),ctspacket,NULL);
                RSTSeenlist.push_back(name);
          }
   // }
 /*   else
    {
        cMessage *mymsg=NULL;
        mymsg=packetwaitinglist.findPacket(rbvtrPacket);
         if(mymsg!=NULL)
         {
          clearMessage(mymsg,rbvtrPacket);
          RBVTR_EV<<"cancel ctspacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
         }
    }*/
}
void RBVTR::processCTSPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()!=getSelfIPAddress())
      {
        cMessage *mymsg=NULL;
        mymsg=packetwaitinglist.findPacket(rbvtrPacket);
        std::string name=rbvtrPacket->getName();
        //RSTSeenlist.push_back(name.substr(3));
         if(mymsg!=NULL)
         {
          clearMessage(mymsg,rbvtrPacket);
          RBVTR_EV<<"cancel ctspacket IP:"<<rbvtrPacket->getdesAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
         }
      }else
      {
        RBVTR_EV<<"send packets IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
        sendDataPacket((rbvtrPacket->getnexthopAddress()),rbvtrPacket->getroads(),rbvtrPacket->getsrcAddress());
      }
}


void RBVTR::processMessage(cPacket * ctrlPacket,IPv4ControlInfo *udpProtocolCtrlInfo)
{
    RBVTRPacket *rbvtrPacket = dynamic_cast<RBVTRPacket *>(ctrlPacket);
    if(rbvtrPacket->getLifetime()<simTime())
    {
        EV_LOG("RBVTR","Time out droped");
        return;
    }
     switch( rbvtrPacket->getPacketType())
    {
     case RBVTR_RD:
               EV_LOG("RBVTR","Process RD");
               processRDPACKET(rbvtrPacket);
               break;
     case RBVTR_RR:
               EV_LOG("RBVTR","Process RR");
               processRRPACKET(rbvtrPacket);
               break;
     case RBVTR_RTS:
                EV_LOG("RBVTR","Process RTS");
                processRTSPACKET(rbvtrPacket);
                break;
     case RBVTR_CTS:
                EV_LOG("RBVTR","Process CTS");
                processCTSPACKET(rbvtrPacket);
                break;

     case RBVTR_RU:
               EV_LOG("RBVTR","Process RU");
               processRUPACKET(rbvtrPacket);
               break;
     default :
         throw cRuntimeError("Unknown packet type");
    }

}

INetfilter::IHook::Result RBVTR::datagramLocalInHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry) {

       EV_LOG("RBVTR","datagramLocalInHook");
    cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
    //Radio80211aControlInfo * cinfo =dynamic_cast<Radio80211aControlInfo*> (datagram->getControlInfo());
   // RBVTR_EV<<"received pow: "<<cinfo->getRecPow()<<endl;

     RBVTRPacket * rbvtrPacket = dynamic_cast<RBVTRPacket *>(networkPacket->getEncapsulatedPacket());
   if (rbvtrPacket) {
       EV_LOG("RBVTR","hello RBVTRPacket");

        networkPacket->decapsulate();
        networkPacket->encapsulate(rbvtrPacket->decapsulate());
        //RBVTR_EV << "datagramLocalInHook " << getSelfIPAddress() << ", target " <<networkPacket-> << endl;

        delete rbvtrPacket;
    }
    return ACCEPT;
}
INetfilter::IHook::Result RBVTR::datagramPostRoutingHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop) {
    EV_LOG("RBVTR","datagramPostRoutingHook");
    const IPv4Address & destination = datagram->getDestAddress();

    if (destination.isMulticast() || destination.isLimitedBroadcastAddress()|| routingTable->isLocalAddress(destination))
               return ACCEPT;
    else{
        if (dynamic_cast<RBVTRPacket *> (dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket()))
        {
            RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>(dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket());
           // RBVTR_EV << "RBVTR   " << rbvtrPacket->getPacketType()<<RBVTR_POST<< endl;
            nextHop=rbvtrPacket->getnexthopAddress().get4();
        }else{
        //  cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
         // RBVTRPacket * rbvtrPacket = dynamic_cast<RBVTRPacket *>(networkPacket->getEncapsulatedPacket());

         // RBVTRPacket * rbvtrdataPacket = createDataPacket(datagram->getDestAddress(), networkPacket->decapsulate());
        //  networkPacket->encapsulate(rbvtrdataPacket);
        // DATASeenlist.SeePacket(rbvtrdataPacket->getsrcAddress(), rbvtrdataPacket->getSeqnum());
         nextHop=IPv4Address::ALLONES_ADDRESS;
        }
    }
    return ACCEPT;
}

INetfilter::IHook::Result RBVTR::datagramForwardHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop)
{
    Enter_Method("datagramForwardHook");
    const IPv4Address & destination = datagram->getDestAddress();

    EV_LOG("RBVTR","datagramForwardHook");
    if (dynamic_cast<RBVTRPacket *> (dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket()))
                 {
                EV_LOG("RBVTR","YES RBVTRPacket");

       // EV_LOG("RBVTR",rbvtrPacket->getPacketType());

     RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>(dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket());
     RBVTR_EV << "RBVTR   " << rbvtrPacket->getPacketType()<<RBVTR_DATA<< endl;
     BroadcastRTS(rbvtrPacket);
     delayPacketlist.addPacket(destination,datagram);
     return QUEUE;
 }else
 {
     return ACCEPT;
 }
}

INetfilter::IHook::Result RBVTR::datagramPreRoutingHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop){
     EV_LOG("RBVTR","datagramPreRoutingHook");
     Enter_Method("datagramPreRoutingHook");
      return ACCEPT;
}

INetfilter::IHook::Result RBVTR::datagramLocalOutHook(IPv4Datagram * datagram, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop){
     EV_LOG("RBVTR","datagramLocalOutHook");
     Enter_Method("datagramLocalOutHook");
     const IPv4Address & destination = datagram->getDestAddress();
     if (destination.isMulticast() || destination.isLimitedBroadcastAddress()|| routingTable->isLocalAddress(destination))
             return ACCEPT;
         else {
             if (!dynamic_cast<RBVTRPacket *> (dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket()))
             {
                 if (routingRoad.hasRoadTable(destination))
                 {
                   RBVTR_EV << "Sending datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress() << endl;
                  cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
                  RBVTRPacket * rbvtrdataPacket = createDataPacket(datagram->getDestAddress(), networkPacket->decapsulate());
                  rbvtrdataPacket->setroads(routingRoad.getRoadTable(destination));
                  rbvtrdataPacket->setdesPosition(routingRoad.getPositionTable(datagram->getDestAddress()));
                  networkPacket->encapsulate(rbvtrdataPacket);
                  //DATASeenlist.SeePacket(rbvtrdataPacket->getsrcAddress(), rbvtrdataPacket->getSeqnum());
                  //nextHop=IPv4Address::ALLONES_ADDRESS;
                  RBVTR_EV << rbvtrdataPacket->getName() << endl;
                   BroadcastRTS(rbvtrdataPacket);
                  }
                 else{
                  RBVTR_EV << "No raod in table :"<<datagram->getDestAddress() <<endl;
                  cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
                  // RBVTRPacket * rbvtrdataPacket = createDataPacket(datagram->getDestAddress(), networkPacket->decapsulate());
                   RBVTRPacket *rbvtrPacket=createRDPacket(  destination,networkPacket->getName());
                  // networkPacket->encapsulate(rbvtrdataPacket);
                  // DATASeenlist.SeePacket(rbvtrdataPacket->getsrcAddress(), rbvtrdataPacket->getSeqnum());
                   RDSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
                   MulticastRIPacket(rbvtrPacket);
                  }
             }else
             {
                 RBVTR_EV << "i am here "<< endl;

                 RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>(dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket());
                 BroadcastRTS(rbvtrPacket);
             }
         }
     delayPacketlist.addPacket(destination,datagram);
     return  QUEUE;
 }

void  RBVTR::test(RBVTRPacket * content)
{

    IPv4ControlInfo *networkProtocolControlInfo = new IPv4ControlInfo();

           networkProtocolControlInfo->setTimeToLive(255);
           networkProtocolControlInfo->setProtocol(IP_PROT_MANET);
           networkProtocolControlInfo->setDestAddr(IPv4Address::LL_MANET_ROUTERS);
           networkProtocolControlInfo->setSrcAddr(getSelfIPAddress());
           UDPPacket *udpPacket = new UDPPacket( content->getName());
           udpPacket->encapsulate(content);
           udpPacket->setSourcePort(269);
           udpPacket->setDestinationPort(269);
           udpPacket->setControlInfo(dynamic_cast<cObject *>(networkProtocolControlInfo));
           sendUDPPacket(udpPacket,0);
}



RBVTRPacket *RBVTR::createRDPacket(IPvXAddress destination,  std::string packetname)
{
    std::string mypacketname=packetname;
    std::string packettype="RD_";
    mypacketname=packettype+mypacketname;
    RBVTRPacket * rBVTRPacket = new RBVTRPacket(mypacketname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    rBVTRPacket->setsrcAddress(getSelfIPAddress());
    rBVTRPacket->setdesAddress(destination);
    rBVTRPacket->setRBVTRPacketType(RBVTR_RD);
    rBVTRPacket->setSeqnum(squmRD++);
    rBVTRPacket->setscrPosition(getSelfPosition());
    rBVTRPacket->setdesPosition(Coord(0,0,0));
    rBVTRPacket->setBitLength(rBVTRPacket->getPacketlength());
    rBVTRPacket->addroad(getRoadID());
    rBVTRPacket->setLifetime(simTime()+RDliftime);
   // rBVTRPacket->encapsulate(content);
    return rBVTRPacket;
}

RBVTRPacket * RBVTR::createDataPacket(IPvXAddress destination, cPacket * content)
{
    std::string packetname=content->getName();
  //  RBVTRPacket * dataPacket = new RBVTRPacket(content->getName());
   // dataPacket->setRBVTRPacketType(RBVTR_DATA);
    std::string packettype="DATA_";
    packetname=packettype+packetname;
    RBVTRPacket * dataPacket = new RBVTRPacket(packetname.c_str());
    dataPacket->setRBVTRPacketType(RBVTR_DATA);
    dataPacket->setsrcAddress(getSelfIPAddress());
    dataPacket->setdesAddress(destination);
    dataPacket->setSeqnum(squmDATA++);
    dataPacket->setLifetime(simTime()+DATAliftime);
    dataPacket->setBitLength(dataPacket->getPacketlength());
    dataPacket->encapsulate(content);
    return dataPacket;
}
void RBVTR::sendDataPacket(const IPvXAddress& target,std::vector<std::string> roads,const IPvXAddress nexthop)
{
    RBVTR_EV << "Completing route discovery, originator " << getSelfIPAddress() << ", target " << target <<", nexthop: " <<nexthop<< endl;

    std::multimap<IPvXAddress, IPv4Datagram *>::iterator lt = delayPacketlist.getlowerbound(target);
    std::multimap<IPvXAddress, IPv4Datagram *>::iterator ut = delayPacketlist.getupperbound(target);
    RBVTRPacket * rbvtrdataPacket;
    // reinject the delayed datagrams
    for (std::multimap<IPvXAddress, IPv4Datagram *>::iterator it = lt; it != ut; it++) {
        IPv4Datagram *datagram = it->second;
        if (!dynamic_cast<RBVTRPacket *> (dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket()))
        {
         cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
         RBVTR_EV << "Sending queued datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress()<<", name: " << networkPacket->getName() << endl;
         rbvtrdataPacket= createDataPacket(datagram->getDestAddress(), networkPacket->decapsulate());
         rbvtrdataPacket->setroads(roads);
         rbvtrdataPacket->setdesPosition(routingRoad.getPositionTable(datagram->getDestAddress()));
         rbvtrdataPacket->setnexthopAddress(nexthop);
         networkPacket->encapsulate(rbvtrdataPacket);
        }else
        {
            rbvtrdataPacket = check_and_cast<RBVTRPacket *>(dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket());
            rbvtrdataPacket->setnexthopAddress(nexthop);
         }
         // DATASeenlist.SeePacket(rbvtrdataPacket->getsrcAddress(), rbvtrdataPacket->getSeqnum());

         networkProtocol->reinjectQueuedDatagram( const_cast<const IPv4Datagram *>(datagram));
    }
    delayPacketlist.removePacket(target);
}

RBVTRPacket *RBVTR::createRRPacket(RBVTRPacket *rbvtrPacket)
{
    std::string packetname=rbvtrPacket->getName();
  //  std::string packettype="RD_";
    packetname=packetname.replace(packetname.begin(),packetname.begin()+2,"RR");
    RBVTRPacket * RRPacket = new RBVTRPacket(packetname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    RRPacket->setsrcAddress(getSelfIPAddress());
    RRPacket->setdesAddress(rbvtrPacket->getsrcAddress());
    RRPacket->setdesPosition(rbvtrPacket->getscrPosition());
    EV<<"Create RR : scr: "<<RRPacket->getsrcAddress()<<" des: "<<RRPacket->getdesAddress()<<endl;
    RRPacket->setRBVTRPacketType(RBVTR_RR);
    RRPacket->setSeqnum(squmRR++);
    RRPacket->setscrPosition(getSelfPosition());
    RRPacket->setroads(rbvtrPacket->getroads());
    RRPacket->setBitLength(RRPacket->getPacketlength());
    RRPacket->setLifetime(simTime()+RRliftime);
   // rBVTRPacket->encapsulate(content);
    return RRPacket;
}
RBVTRPacket *RBVTR::createRTSPacket(RBVTRPacket *rbvtrPacket)
{
    std::string packetname=rbvtrPacket->getName();
  //  std::string packettype="RD_";
    //packetname=packetname.replace(packetname.begin(),packetname.begin()+2,"RTS");
    packetname="RTS_"+packetname;
    RBVTRPacket * RTSPacket = new RBVTRPacket(packetname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    RTSPacket->setsrcAddress(getSelfIPAddress());
    RTSPacket->setdesAddress(rbvtrPacket->getdesAddress());
    RTSPacket->setdesPosition(rbvtrPacket->getdesPosition());
    EV<<"Create RTS : scr: "<<RTSPacket->getsrcAddress()<<" des: "<<RTSPacket->getdesAddress()<<endl;
    RTSPacket->setRBVTRPacketType(RBVTR_RTS);
    RTSPacket->setSeqnum(squmRTS++);
    RTSPacket->setscrPosition(getSelfPosition());
    RTSPacket->setroads(rbvtrPacket->getroads());
    RTSPacket->setBitLength(RTSPacket->getPacketlength());
    RTSPacket->setLifetime(simTime()+RRliftime);
   // rBVTRPacket->encapsulate(content);
    return RTSPacket;
}
RBVTRPacket *RBVTR::createCTSPacket(RBVTRPacket *rbvtrPacket)
{
    std::string packetname=rbvtrPacket->getName();
  //  std::string packettype="RD_";
    packetname=packetname.replace(packetname.begin(),packetname.begin()+1,"C");
    RBVTRPacket * CTSPacket = new RBVTRPacket(packetname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    CTSPacket->setsrcAddress(getSelfIPAddress());
    CTSPacket->setnexthopAddress(rbvtrPacket->getdesAddress());
    CTSPacket->setdesAddress(rbvtrPacket->getsrcAddress());
    CTSPacket->setdesPosition(rbvtrPacket->getdesPosition());
    EV<<"Create CTS : scr: "<<CTSPacket->getsrcAddress()<<" des: "<<CTSPacket->getdesAddress()<<endl;

    CTSPacket->setRBVTRPacketType(RBVTR_CTS);
    CTSPacket->setSeqnum(rbvtrPacket->getSeqnum());
    CTSPacket->setscrPosition(rbvtrPacket->getscrPosition());
    CTSPacket->setroads(rbvtrPacket->getroads());
    CTSPacket->setBitLength(CTSPacket->getPacketlength());
    CTSPacket->setLifetime(simTime()+RRliftime);
   // rBVTRPacket->encapsulate(content);
    return CTSPacket;
}
RBVTRPacket *RBVTR::createRUPacket(IPvXAddress destination,  std::vector<std::string>roads)
{
     std::string mypacketname="RU_"+getHostName();
    RBVTRPacket * rBVTRPacket = new RBVTRPacket(mypacketname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    rBVTRPacket->setsrcAddress(getSelfIPAddress());
    rBVTRPacket->setdesAddress(destination);
    rBVTRPacket->setRBVTRPacketType(RBVTR_RU);
    rBVTRPacket->setSeqnum(squmRU++);
    rBVTRPacket->setscrPosition(getSelfPosition());
    rBVTRPacket->setroads(roads);
    rBVTRPacket->setLifetime(simTime()+RUliftime);
    rBVTRPacket->setBitLength(rBVTRPacket->getPacketlength());
   // rBVTRPacket->encapsulate(content);
    return rBVTRPacket;
}
/*void RBVTR::handleMessage(cMessage *message)
{
    EV_LOG("RBVTR","handleMessage");

}*/
