/*
 * RBVTRPacket.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: chengli
 */

#include <RBVTR/RBVTR.h>
#define RBVTR_EV EV << "RBVTR at " << getHostName() << " "
#define LOG_EV inFile <<"#"<<EventNumber()<<"  "<<simTime()<< " " << getHostName() << " "
#include "TraCIMobility.h"
#include "GPSR_m.h"
#include "Radio80211aControlInfo_m.h"
Define_Module(RBVTR);
double maxre=0;

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
        EV_LOG(getHostName()+"road "+myroad);
        IPSocket socket(gate("ipOut"));
        socket.registerProtocol(IP_PROT_MANET);
        squmRD=0;
        squmRR=0;
        squmRU=0;
        squmDATA=0;
        squmRTS=0;
        squmRE=0;

        RDliftime= par("RDliftime");
        RRliftime= par("RRliftime");
        DATAliftime= par("DATAliftime");
        REwaittime= par("REwaittime");
        nextRUtimer= par("nextRUtimer");
        RUliftime= par("RUliftime");
        routingTable = check_and_cast<IRoutingTable *>(getModuleByPath(par("routingTableModule")));
        RUTimer = new cMessage("RUTimer");
          Tmax= par("Tmax");
          dopt= par("dopt");
          dmax= par("dmax");
          maxRebroadcastTime= par("maxRebroadcastTime");
          a1= par("a1");
          a2= par("a2");
          a3= par("a3");
        //reBroadcastRDTimer = new cMessage("ReBroadcastRDTimer");;
          RouteInterface::protocalname="RBVTR";
    }else{
    if (stage == 4)
        {
        oldroadID=getRoadID();
        RouteInterface::configureInterfaces(par("interfaces"));
        EV_LOG(getHostName());
        nb->subscribe(this, NF_LINK_FULL_PROMISCUOUS);
        scheduleAt(simTime() + nextRUtimer, RUTimer);
        globalPositionTable.setHostName(getSelfIPAddress(),getHostName());
        }
    }
    //: public cSimpleModule, public ILifecycle, public INotifiable, public INetfilter::IHookRouteInterface::initialize();
 }
std::string RBVTR::getRoadID()
{
    string myroad =RouteInterface::getRoadID();
    if(myroad.length()!=8)
    {
        return oldroadID;
    }else
    {
        return myroad;
    }
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
       //  inFile<<" ???? £º"<<distence;

     }else
     {
        distence = (srcPosition-desPosition).length()-(desPosition - getSelfPosition()).length();
       // inFile<<" with real distence £º"<<distence;
        if(distence<0)
            distence=dmax;
        inFile<<" with distence £º"<<distence;
     }
    double parmax=(pow(dmax,a1)*pow(CaculateF(dopt),a2)*pow(0.000000001,a3));
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
    //inFile<<" with real waiting time "<<result;
    if (result <0)
        result =Tmax;
    inFile<<" with waiting time "<<result<<endl;
    //inFile<<" f "<<f <<" p "<<p<<" ss "<<ss<<" parmax  "<<parmax<<endl;
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

void RBVTR::scheduleRErunoutTimer(simtime_t holdingtime)
{
    EV_LOG( "scheduleRErunoutTimer" );
    scheduleAt(holdingtime, RETimer);
}
void RBVTR::scheduleReBroadcastRDTimer(simtime_t holdingtime,RBVTRPacket *rbvtrPacket,IPv4Datagram * datagram)
{
    EV_LOG( "Scheduling ReBroadcast timer" );
    std::string timername="reBroadcastRDTimer_";
    if (rbvtrPacket->getPacketType() ==RBVTR_RR)
    {
     timername="reBroadcastRRTimer_";
    }
    else
    {
        if (rbvtrPacket->getPacketType() ==RBVTR_DATA)
            {
            timername="reBroadcastDATATimer_";
            }else
            {
                if (rbvtrPacket->getPacketType() ==RBVTR_RE)
                  {
                timername="reBroadcastRETimer_";
                 }
            }
    }
    timername=timername+rbvtrPacket->getName();
    EV_LOG( timername);
    cMessage * reBroadcastRDTimer = new cMessage(timername.c_str());
    packetwaitinglist.addPacket(reBroadcastRDTimer,rbvtrPacket,datagram);
    /* std::vector<cPacket*>queuerts= packetwaitinglist.getAllcPackets();
    LOG_EV<<"add packet "<<rbvtrPacket->getName()<<" num: "<<queuerts.size()<<endl;
     for(int i=0;i<queuerts.size();i++)
     {
         LOG_EV<<queuerts[i]->getName()<<endl;
     }*/
    scheduleAt(simTime() + holdingtime, reBroadcastRDTimer);
}
void RBVTR::processRETimer()
{

        Queuedeslist.pop();
        if(Queuedeslist.size()!=0)
        {
            scheduleRErunoutTimer(Queuedeslist.front().second);
        }
}

void RBVTR::processRUTimer(simtime_t timer)
{
    //EV_LOG( "Scheduling RU timer" );
    getRoadID();
    if(oldroadID!=getRoadID()&&oldroadID.length()==8)
    {
       std::vector<IPvXAddress> IPs= routingRoad.getAllIP();
       for(int i=0;i<IPs.size();i++)
       {
          // std::vector<std::string>roads=routingRoad.getRoadTable(IPs[i]);
           EV_LOG( "oldroadID " +oldroadID+" new road "+getRoadID());
           routingRoad.findnewRoadid(IPs[i],getRoadID());
           for(int j=0;j<routingRoad.getRoadTable(IPs[i]).size();j++)
           {
               EV<< "road "<<routingRoad.getRoadTable(IPs[i])[j]<<endl ;
               std::cout<< "road   "<<routingRoad.getRoadTable(IPs[i])[j]<<endl ;
           }
           RBVTRPacket *rbvtrPacket=createRUPacket(  IPs[i],routingRoad.getRoadTable(IPs[i]));
           RUSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
           sendRIPacket(rbvtrPacket,rbvtrPacket->getdesAddress(),255,0);
       }
       oldroadID=getRoadID();
    }
    scheduleAt(simTime() + timer, RUTimer);
 }
void RBVTR::processSelfMessage(cMessage * message)
{
    if(message==RETimer)
        processRETimer();
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
     cancelEvent(message);
     //cancelEvent(message);
     if(rbvtrPacket!=NULL)
     {
     networkProtocol->dropQueuedDatagram(dynamic_cast<const IPv4Datagram *>(rbvtrPacket));
     }
 }

void RBVTR::processRDTimer(cMessage * message,RBVTRPacket *rbvtrPacket,const IPv4Datagram * pakcet)
{
    EV_LOG( "processRDTimer" );
    if(rbvtrPacket->getPacketType()!=RBVTR_DATA)
    {
        if (rbvtrPacket->getPacketType() ==RBVTR_RTS)
       {
            if(std::string(rbvtrPacket->getName()).find("RU")==std::string::npos)
            LOG_EV<<"I reBroadcast RTS: "<<rbvtrPacket->getName()<<endl;
            BroadcastRTS(rbvtrPacket);
       }else
       {
           MulticastRIPacket(rbvtrPacket);
       }
    }else
    {
        if(pakcet!=NULL)
        {
       networkProtocol->reinjectQueuedDatagram( const_cast<const IPv4Datagram *>(pakcet));
        }else
        {
            EV_LOG("I got a NULL");
        }
    }
}
void RBVTR::BroadcastRTS(RBVTRPacket * rbvtrPacket)
{
   Enter_Method("BroadcastRTS");
   EV_LOG("BroadcastRTS");
   EV_LOG("create RTS for "+std::string(rbvtrPacket->getName()));
   RBVTRPacket *RTSPacket=createRTSPacket(rbvtrPacket);
   std::string name=RTSPacket->getName();
   RSTSeenlist.push_back(name);
   EV_LOG("broadcast RTS");
   if(std::string(RTSPacket->getName()).find("RU")==std::string::npos)
   {
   LOG_EV<<"BroadcastRTS : "<<RTSPacket->getName()<<" at "<<getRoadID()<<endl;
   }
   //RBVTRPacket *RTSPacketcopy=createRTSPacket(RTSPacket);
   MulticastRIPacket(RTSPacket);
   if(rtsrebroadcastlist.find(rbvtrPacket->getName())==rtsrebroadcastlist.end())
      {
          rtsrebroadcastlist[rbvtrPacket->getName()]=0;
      }else
      {
          rtsrebroadcastlist[rbvtrPacket->getName()]=rtsrebroadcastlist[rbvtrPacket->getName()]+1;
      }
   if(rbvtrPacket->getPacketType()==RBVTR_DATA)
   {
       RBVTRPacket *repacket=createREPacket(rbvtrPacket);
       scheduleReBroadcastRDTimer(0.5,repacket,NULL);
   }  //RTSlist.addPacket(RTSPacket->getSeqnum(),rbvtrPacket,NULL);
}

void RBVTR::processRDPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
         {
            EV_LOG("I got a RD");


             std::vector<std::string > roads=rbvtrPacket->getroads();
             std::vector<std::string > temproad;
             if(std::find(roads.begin(),roads.end(),getRoadID())==roads.end())
              {
                 roads.push_back(getRoadID());
              }
             for(int i=0;i<roads.size();i++)
             {
                 RBVTR_EV<<"roads: "<<roads[i]<<endl;
                 std::cout<<"roads: "<<roads[i]<<endl;
                 //LOG_EV<<"old roads: "<<roads[i]<<endl;
                 temproad.push_back(roads[i]);
                 if(i+1<roads.size()&&roads[i]==roads[i+1])
                 {
                     i++;
                 }
             }
             roads=temproad;
             temproad.clear();
             /*for(int i=0;i<roads.size();i++)
            {
                RBVTR_EV<<"roads: "<<roads[i]<<endl;
                std::cout<<"roads: "<<roads[i]<<endl;
                LOG_EV<<"old roads: "<<roads[i]<<endl;
            }*/
             if(roads.size()>2)
              {
                 for(int i=0;i<roads.size()-1;i++)
                 {
                  std::string intersection =getRoadIntersection(roads[i],roads[i+1]);
                  if(intersection==std::string("none"))
                  {
                    std::string newroad=  caculateConnection(roads[i],roads[i+1]);
                    if(newroad==std::string("none"))
                        return;
                    //std::string oldroad =temproad[rbvtrPacket->roads.size()-1];
                    //rbvtrPacket->roads[rbvtrPacket->roads.size()-1]=newroad;
                    //rbvtrPacket->roads.push_back(oldroad);
                    //LOG_EV<<"old "<<roads[i]<<" "<<roads[i+1]<<" add "<<newroad<<endl;
                    temproad.push_back(roads[i]);
                    temproad.push_back(newroad);
                  }
                  else
                  {
                     // LOG_EV<<"old "<<roads[i]<<" "<<roads[i+1]<<" the intersection  "<<intersection<<endl;
                      temproad.push_back(roads[i]);
                  }
                 }
                 temproad.push_back(roads[roads.size()-1]);
                 roads=temproad;
                 /*for(int i=0;i<roads.size();i++)
                     {
                         RBVTR_EV<<"roads: "<<roads[i]<<endl;
                         std::cout<<"roads: "<<roads[i]<<endl;
                         LOG_EV<<"roads: "<<roads[i]<<endl;
                     }*/
                 temproad.clear();
                 temproad.push_back(roads[0]);
                 temproad.push_back(roads[1]);
                 for(int i=2;i<roads.size();i++)
                 {
                     for(int j=i-2;j<i-1;j++)
                      {
                         if(getRoadIntersection(roads[j],roads[i])!=std::string("none"))
                              {
                              //LOG_EV<<"del "<<roads[i]<<" "<<roads[j]<<" the intersection  "<<temproad[temproad.size()-1]<<endl;
                              temproad[temproad.size()-1]=roads[i];
                              }
                         else
                         {
                             temproad.push_back(roads[i]);
                         }
                      }
                 }
              }
             else
              {
                 temproad.push_back(getRoadID());
              }


             rbvtrPacket->roads=temproad;
             std::vector <std::string> routingroad;
              for(int i=0;i<rbvtrPacket->getroads().size();i++)
              {
                   routingroad.push_back( rbvtrPacket->getroads()[rbvtrPacket->getroads().size()-1-i]);
               }
             if(routingRoad.addRoadTable(rbvtrPacket->getsrcAddress(),routingroad,rbvtrPacket->getscrPosition()))
             {
             //reply RR
                 LOG_EV<<"this RD for me and reply RR"<<endl;

                  RBVTRPacket *RrPacket=createRRPacket(  rbvtrPacket);
                  //RBVTR_EV << "Add raod in table :"<<rbvtrPacket->getsrcAddress()<<endl;
                  //std::cout<< "Add raod in table :"<<rbvtrPacket->getsrcAddress()<<endl;
                  for(int i=0;i<rbvtrPacket->getroads().size();i++)
                      {
                          RBVTR_EV<<"roadlist: "<<rbvtrPacket->getroads()[i]<<endl;
                          std::cout<<"roadlist: "<<rbvtrPacket->getroads()[i]<<endl;
                          LOG_EV<<"roadlist: "<<rbvtrPacket->getroads()[i]<<endl;
                      }
                 sendRIPacket(RrPacket,RrPacket->getdesAddress(),255,0);
             }
           }
         else
         {
             if((!RDSeenlist.isSeenPacket(rbvtrPacket->getsrcAddress(),rbvtrPacket->getSeqnum())))
             {
                 //LOG_EV<<" got a RD from :"<<globalPositionTable.getHostName(rbvtrPacket->nexthop_ip)<<endl;
                 RBVTR_EV<<"no seen RDpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<" last road: "<<rbvtrPacket->getroads().back()<<" my road: "<<getRoadID()<<endl;
                 std::cout<<"no seen RDpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<" last road: "<<rbvtrPacket->getroads().back()<<" my road: "<<getRoadID()<<endl;
                 std::cout<<"got RDpacket from:"<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<"  to: "<<getHostName()<<endl;
                 std::vector<std::string> routingroads=rbvtrPacket->getroads();

                 for (int i=0 ;i<routingroads.size();i++)
                 {
                     EV_LOG("road in packet: "+routingroads[i]);
                 }
                 if(routingroads.size()==0||routingroads.back()!=getRoadID())
                 {
                     if(routingroads.size()>=2&&hasIntersection(getRoadID(),getRoadIntersection(routingroads[routingroads.size()-2],routingroads[routingroads.size()-1])))
                     {
                         EV_LOG("Delete intersection");
                         rbvtrPacket->getroads()[routingroads.size()-1]=getRoadID();
                     }else
                     {
                         EV_LOG("add intersection "+getRoadID());
                         rbvtrPacket->addroad(getRoadID());
                     }
                 }
                 EV_LOG("My road is:  "+getRoadID());
               routingroads=rbvtrPacket->getroads();
               for (int i=0 ;i<routingroads.size();i++)
               {
                   EV_LOG("new road in packet: "+routingroads[i]);
               }
               rbvtrPacket->nexthop_ip=getSelfIPAddress();
                // scheduleReBroadcastRDTimer(CaculateHoldTime(rbvtrPacket->getscrPosition(),rbvtrPacket->getdesPosition()),rbvtrPacket,NULL);
               scheduleReBroadcastRDTimer(CaculateHoldTime(rbvtrPacket->getsenderPosition()),rbvtrPacket,NULL);
               rbvtrPacket->setsenderPosition(getSelfPosition());
               RDSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
             }
             else
             {
                 if(rbvtrPacket->getroads().back()!=getRoadID())//std::find(rbvtrPacket->getroads().begin(),rbvtrPacket->getroads().end(),getRoadID())==rbvtrPacket->getroads().end())
                 {
                    // LOG_EV<<" drop a RD from :"<<globalPositionTable.getHostName(rbvtrPacket->nexthop_ip)<<endl;
                     RBVTR_EV<<"seen a RDpacket IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
                   //  LOG_EV<<"i've seen this RDpacket IP:"<<rbvtrPacket->getName()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
                     cMessage *mymsg=NULL;
                     mymsg=packetwaitinglist.findPacket(rbvtrPacket);
                     if(mymsg!=NULL)
                       clearMessage(mymsg,rbvtrPacket);
                 }
             }
         }
}
void RBVTR::processREPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
            {
        LOG_EV<<"I got a RE"<<endl;
    EV_LOG("I got a RE");
    RBVTR_EV <<(rbvtrPacket->getdesAddress())<<"           "<<getSelfIPAddress()<<endl;
    Queuedeslist.push(std::pair<IPvXAddress ,simtime_t>(rbvtrPacket->getdesAddress(),simTime()+REwaittime));
   // for(int i=0;i<sentPacketlist.size();i++)
   //    {
           // string name =string(rbvtrPacket->getName());
            //LOG_EV<<sentPacketlist[i].second->getName()<<" my name "<<name.substr(7)<<endl;

          /// if(sentPacketlist[i].second->getName()== name.substr(7))
           //{
           //    sendRIPacket(sentPacketlist[i].second,sentPacketlist[i].first,255,0);
         //  }
    //   }

   }
 else
 {
     EV_LOG("ERROR:Others' RE");
 }
}
void RBVTR::processRRPACKET(RBVTRPacket * rbvtrPacket)
{
    if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
         {
            EV_LOG("I got a RR");
            LOG_EV<<"i got RR from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<endl;
            RBVTR_EV <<(rbvtrPacket->getdesAddress())<<"           "<<getSelfIPAddress()<<endl;
            if(routingRoad.addRoadTable(rbvtrPacket->getsrcAddress(),rbvtrPacket->getroads(),rbvtrPacket->getscrPosition()))
            {
                RBVTR_EV << "Add raod in table :"<<rbvtrPacket->getsrcAddress()<<endl;
            // BroadcastRTS(rbvtrPacket);
            for(int i=0;i<rbvtrPacket->getroads().size();i++)
                {
                    RBVTR_EV<<"roadlist: "<<rbvtrPacket->getroads()[i]<<endl;
                    std::cout<<"roadlist: "<<rbvtrPacket->getroads()[i]<<endl;
                }
            sendDataPacket((rbvtrPacket->getsrcAddress()),rbvtrPacket->getroads());
            }
           // RRSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());

           }
         else
         {
             EV_LOG("ERROR:Others' RR");
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

                   scheduleReBroadcastRDTimer(CaculateHoldTime(rbvtrPacket->getsenderPosition(),rbvtrPacket->getdesPosition()),rbvtrPacket,NULL);
                   rbvtrPacket->setsenderPosition(getSelfPosition());
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
          RBVTR_EV<<"RTS: "<<name<<endl;

          for(int i=0;i<RSTSeenlist.size();i++)
          {
              RBVTR_EV<<"RTSSEEN: "<<RSTSeenlist[i]<<endl;
          }
          EV_LOG("myroad is: "+getRoadID());

          if(rbvtrPacket->getdesAddress()==getSelfIPAddress())
          {
              LOG_EV<<"i got Rts from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<endl;
              RBVTR_EV<<"got RTS IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
              RBVTRPacket *ctspacket=  createCTSPacket( rbvtrPacket);
              scheduleReBroadcastRDTimer(0.00000001,ctspacket,NULL);
              return;
          }
         double distence = (rbvtrPacket->getscrPosition()-rbvtrPacket->getdesPosition()).length()-(rbvtrPacket->getdesPosition() - getSelfPosition()).length();

          if((std::find(routingroad.begin(),routingroad.end(),getRoadID())!=routingroad.end())&&(std::find(RSTSeenlist.begin(),RSTSeenlist.end(),(name))==RSTSeenlist.end()))
          {
              std::vector<std::string> passroad=rbvtrPacket->passedroads;
              for(int i=0;i<passroad.size();i++)
               {
                  LOG_EV<<"passroad: "<<passroad[i]<<endl;
               }
              if(std::find(passroad.begin(),passroad.end(),getRoadID())==passroad.end())
              {
                LOG_EV<<"i got Rts from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress());
                RBVTR_EV<<"got RTS IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
                RBVTRPacket *ctspacket=  createCTSPacket( rbvtrPacket);
                scheduleReBroadcastRDTimer(CaculateHoldTime(ctspacket->getscrPosition(),ctspacket->getdesPosition()),ctspacket,NULL);
              }else
              {
                  LOG_EV<<"i drop Rts from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<" with distence : "<<distence<<endl;
              }
              //  RSTSeenlist.push_back(name);
          }else
          {
              EV_LOG("SEEN this RTS: "+name);
              if(RouteInterface::getRoadID().length()==3&&hasIntersection(getRoadID(),RouteInterface::getRoadID()))
              {
                  for(int i=1;i<routingroad.size()-1;i++)
                  {
                      if(routingroad[i].find(RouteInterface::getRoadID())!=std::string::npos)
                      {
                          LOG_EV<<"i got Rts from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<" with distence : "<<distence<<" my road: "<<RouteInterface::getRoadID()<<endl;
                          RBVTRPacket *ctspacket=  createCTSPacket( rbvtrPacket);
                          scheduleReBroadcastRDTimer(CaculateHoldTime(ctspacket->getsenderPosition(),ctspacket->getdesPosition()),ctspacket,NULL);
                          return;
                      }
                  }
                  LOG_EV<<"i drop Rts from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<" with distence : "<<distence<<" my road: "<<RouteInterface::getRoadID()<<endl;

                        /* std::vector<std::string> verticalroad;
                          for(int i=0;i<routingroad.size()-1;i++)
                              {
                                  RBVTR_EV<<"roadlist: "<<routingroad[i]<<endl;
                                  //std::string intersection =getRoadIntersection(roads[i],roads[i+1]);
                                  if(isRoadVertical(routingroad[i],routingroad[i+1]))
                                  {
                                      std::string aimroad;
                                      if(std::string(rbvtrPacket->getName()).find("RR")==std::string::npos)
                                      {
                                          aimroad=routingroad[i];
                                      }else
                                      {
                                          aimroad=routingroad[i+1];
                                      }
                                      if(!isRoadVertical(aimroad,getRoadID())&&getRoadIntersection(aimroad,getRoadID())!=std::string("none"))
                                      {
                                          LOG_EV<<"i got Rts from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<" with distence : "<<distence<<" my road: "<<RouteInterface::getRoadID()<<endl;
                                          RBVTRPacket *ctspacket=  createCTSPacket( rbvtrPacket);
                                          scheduleReBroadcastRDTimer(Tmax,ctspacket,NULL);
                                          return;
                                      }
                                  }
                              }*/

                  }
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
    EV_LOG("Process CTS: "+std::string(rbvtrPacket->getName())+" des: "+globalPositionTable.getHostName(rbvtrPacket->getdesAddress()));

    if(rbvtrPacket->getdesAddress()!=getSelfIPAddress())
      {
        cMessage *mymsg=NULL;
        mymsg=packetwaitinglist.findPacket(rbvtrPacket->getName());
        //RSTSeenlist.push_back(name.substr(3));
         if(mymsg!=NULL)
         {
          clearMessage(mymsg,rbvtrPacket);
          RBVTR_EV<<"cancel ctspacket IP:"<<rbvtrPacket->getdesAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
          std::cout<<"cancel ctspacket IP:"<<rbvtrPacket->getdesAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
         }
      }else
      {
        LOG_EV<<"i got CTS from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<endl;
        RBVTR_EV<<"i got CTS from "<<globalPositionTable.getHostName(rbvtrPacket->getsrcAddress())<<endl;
        RBVTR_EV<<"send packets IP:"<<rbvtrPacket->getsrcAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
        delRTSreBroadcast(rbvtrPacket->getName());
        sendDataPacket((rbvtrPacket->getnexthopAddress()),rbvtrPacket->getroads(),rbvtrPacket->getsrcAddress());
      }
}

void RBVTR::delRTSreBroadcast(string name)
{
    std::string packetname=name.replace(name.begin(),name.begin()+3,"RE");
    //packetname="RE_"+packetname;
    cMessage *mymsg=NULL;
    mymsg=packetwaitinglist.findPacket(packetname);
    //RSTSeenlist.push_back(name.substr(3));
     if(mymsg!=NULL)
     {
         LOG_EV<<"dele RE "<<packetname<<endl;
      clearMessage(mymsg,NULL);
      //LOG_EV<<"cancel RTS from "<<mymsg->getName()<<endl;

      //RBVTR_EV<<"cancel rtspacket IP:"<<rbvtrPacket->getdesAddress()<<"  SQUM: "<<rbvtrPacket->getSeqnum()<<endl;
     }
     else
     {
         std::vector<cPacket*>queuerts= packetwaitinglist.getAllcPackets();

         LOG_EV<<"can't find RE "<<packetname<<" num: "<<queuerts.size()<<endl;
         for(int i=0;i<queuerts.size();i++)
         {
             LOG_EV<<queuerts[i]->getName()<<endl;
         }
     }
   }

void RBVTR::processMessage(cPacket * ctrlPacket,IPv4ControlInfo *udpProtocolCtrlInfo)
{
    RBVTRPacket *rbvtrPacket = dynamic_cast<RBVTRPacket *>(ctrlPacket);
    if(rbvtrPacket->getLifetime()<simTime())
    {
        EV_LOG("Time out droped");
        return;
    }
     switch( rbvtrPacket->getPacketType())
    {
     case RBVTR_RD:
               EV_LOG("Process RD");
               processRDPACKET(rbvtrPacket);
               break;
     case RBVTR_RR:
               EV_LOG("Process RR");
               processRRPACKET(rbvtrPacket);
               break;
     case RBVTR_RE:
               EV_LOG("Process RE");
               processREPACKET(rbvtrPacket);
               break;
     case RBVTR_RTS:
                EV_LOG("Process RTS");
                processRTSPACKET(rbvtrPacket);
                break;
     case RBVTR_CTS:
                processCTSPACKET(rbvtrPacket);
                break;

     case RBVTR_RU:
               EV_LOG("Process RU");
               processRUPACKET(rbvtrPacket);
               break;
     default :
         throw cRuntimeError("Unknown packet type");
    }

}

INetfilter::IHook::Result RBVTR::datagramLocalInHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry) {

       EV_LOG("datagramLocalInHook");
    cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
    //Radio80211aControlInfo * cinfo =dynamic_cast<Radio80211aControlInfo*> (datagram->getControlInfo());
   // RBVTR_EV<<"received pow: "<<cinfo->getRecPow()<<endl;

     RBVTRPacket * rbvtrPacket = dynamic_cast<RBVTRPacket *>(networkPacket->getEncapsulatedPacket());
   if (rbvtrPacket) {
       EV_LOG("hello RBVTRPacket");

        networkPacket->decapsulate();
        networkPacket->encapsulate(rbvtrPacket->decapsulate());
        //RBVTR_EV << "datagramLocalInHook " << getSelfIPAddress() << ", target " <<networkPacket-> << endl;

        delete rbvtrPacket;
    }
    return ACCEPT;
}
INetfilter::IHook::Result RBVTR::datagramPostRoutingHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop) {
    EV_LOG("datagramPostRoutingHook");
    const IPv4Address & destination = datagram->getDestAddress();

    if (destination.isMulticast() || destination.isLimitedBroadcastAddress()|| routingTable->isLocalAddress(destination))
               return ACCEPT;
    else{
        if(dynamic_cast<RBVTRPacket *>( (dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))
        {
            RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>( (dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket());
            if(rbvtrPacket->nexthop_ip.get4()==IPv4Address::UNSPECIFIED_ADDRESS&&rbvtrPacket->getPacketType()==RBVTR_DATA)
                {
                   BroadcastRTS(rbvtrPacket);
                   tempdelayPacketlist.addPacket(destination,datagram);
                   //delayPacketlist.addPacket
                   return QUEUE;
                }else
                {
                    nextHop=rbvtrPacket->nexthop_ip.get4();
                    return ACCEPT;
                }
        }
        if (dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket()))
        {
            RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket());
           // RBVTR_EV << "RBVTR   " << rbvtrPacket->getPacketType()<<RBVTR_POST<< endl;

            nextHop=rbvtrPacket->nexthop_ip.get4();
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

    EV_LOG("datagramForwardHook");
    if(dynamic_cast<RBVTRPacket *>( (dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))
    {
        EV_LOG("YES RBVTRPacket");
           RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>( (dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket());
           RBVTR_EV << "RBVTR   " <<rbvtrPacket->getFullName()<<"   "<< rbvtrPacket->getPacketType()<<RBVTR_DATA<< endl;
           BroadcastRTS(rbvtrPacket);
           delayPacketlist.addPacket(destination,datagram);
           return QUEUE;
    }
    if (dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket()))
                 {
                EV_LOG("YES RBVTRPacket");
                 RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket());
                 RBVTR_EV << "RBVTR   " <<rbvtrPacket->getFullName()<<"   "<< rbvtrPacket->getPacketType()<<RBVTR_DATA<< endl;
                 if(rbvtrPacket->getcurrentroad()!=getRoadID())
                 {
                     std::vector<std::string> passroad=rbvtrPacket->passedroads;
                     if(std::find(passroad.begin(),passroad.end(),rbvtrPacket->getcurrentroad())==passroad.end())
                      {
                          rbvtrPacket->passedroads.push_back(rbvtrPacket->getcurrentroad());
                      }
                     rbvtrPacket->setcurrentroad(getRoadID());
                 }

                 BroadcastRTS(rbvtrPacket);
                 delayPacketlist.addPacket(destination,datagram);
                 return QUEUE;
             }else
             {
                 return ACCEPT;
             }
}

INetfilter::IHook::Result RBVTR::datagramPreRoutingHook(IPv4Datagram * datagram, const InterfaceEntry * inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop){
     EV_LOG("datagramPreRoutingHook");
     Enter_Method("datagramPreRoutingHook");
      return ACCEPT;
}

INetfilter::IHook::Result RBVTR::datagramLocalOutHook(IPv4Datagram * datagram, const InterfaceEntry *& outputInterfaceEntry, IPv4Address & nextHop){
     EV_LOG("datagramLocalOutHook");
     Enter_Method("datagramLocalOutHook");
     const IPv4Address & destination = datagram->getDestAddress();
     if (destination.isMulticast() || destination.isLimitedBroadcastAddress()|| routingTable->isLocalAddress(destination))
             return ACCEPT;
         else {
             EV_LOG("i am here 2"+std::string(datagram->getName()));
             if(std::string(datagram->getName()).find("ICMP")!=std::string::npos)
             {
                 cout << "got "<<datagram->getName() <<endl;
                 LOG_EV << "got "<<datagram->getName() <<endl;
                 return DROP;
             }
             if (dynamic_cast<RBVTRPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))//||dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket()))
                          {
                              EV_LOG("i am here 3"+std::string(datagram->getName()));
                          }else
                          {
                              if (dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket()))
                              {
                                  EV_LOG("i am here 4"+std::string(datagram->getName()));
                              }
                          }

             if (dynamic_cast<RBVTRPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket())||dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket()))
             {
                 RBVTR_EV << "i am here "<< endl;
                 cout << "i am here "<< endl;

                // RBVTRPacket * rbvtrPacket = check_and_cast<RBVTRPacket *>(dynamic_cast<cPacket *>(datagram)->getEncapsulatedPacket());

                 RBVTRPacket * rbvtrPacket ;
                 if (!dynamic_cast<RBVTRPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket())){
                     rbvtrPacket=dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket());
                 }else
                 {
                     rbvtrPacket=dynamic_cast<RBVTRPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket());
                 }
                 BroadcastRTS(rbvtrPacket);
             }else
             {
                 if (routingRoad.hasRoadTable(destination))
                      {
                        RBVTR_EV << "Sending datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress() << endl;
                        std::cout << "Sending datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress() << endl;
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
                       cout << "No raod in table :"<<datagram->getDestAddress() <<endl;

                       cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
                       // RBVTRPacket * rbvtrdataPacket = createDataPacket(datagram->getDestAddress(), networkPacket->decapsulate());
                        RBVTRPacket *rbvtrPacket=createRDPacket(  destination,networkPacket->getName());
                       // networkPacket->encapsulate(rbvtrdataPacket);
                       // DATASeenlist.SeePacket(rbvtrdataPacket->getsrcAddress(), rbvtrdataPacket->getSeqnum());
                        RDSeenlist.SeePacket(rbvtrPacket->getsrcAddress(), rbvtrPacket->getSeqnum());
                        MulticastRIPacket(rbvtrPacket);
                       }

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
    mypacketname=packettype+mypacketname+"_"+getHostName()+"_"+std::to_string(squmRD++);
    RBVTRPacket * rBVTRPacket = new RBVTRPacket(mypacketname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    rBVTRPacket->setsrcAddress(getSelfIPAddress());
    rBVTRPacket->setdesAddress(destination);
    rBVTRPacket->setRBVTRPacketType(RBVTR_RD);
    rBVTRPacket->setSeqnum(squmRD);
    rBVTRPacket->setscrPosition(getSelfPosition());
    rBVTRPacket->setdesPosition(Coord(0,0,0));
    rBVTRPacket->setBitLength(rBVTRPacket->getPacketlength());
    rBVTRPacket->addroad(getRoadID());
    rBVTRPacket->nexthop_ip=getSelfIPAddress();
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
    packetname=packettype+packetname+"_"+getHostName()+"_"+std::to_string(squmDATA++);
    LOG_EV<<"createDataPacket: "<<packetname<<endl;
    RBVTRPacket * dataPacket = new RBVTRPacket(packetname.c_str());
    dataPacket->setRBVTRPacketType(RBVTR_DATA);
    dataPacket->setsrcAddress(getSelfIPAddress());
    dataPacket->setdesAddress(destination);
    dataPacket->setSeqnum(squmDATA);
    dataPacket->setLifetime(simTime()+DATAliftime);
    dataPacket->setcurrentroad(getRoadID());
    //dataPacket->addpassedroad(getRoadID());
    dataPacket->setBitLength(dataPacket->getPacketlength());
    dataPacket->encapsulate(content);
    RBVTR_EV << "create Data packet" << packetname<<" for "<<content->getName()<< " , target " << destination << endl;
    return dataPacket;
}

void RBVTR::sendDataPacket(const IPvXAddress& target,std::vector<std::string> roads)
{
    sendDataPacket(target,roads,IPv4Address::UNSPECIFIED_ADDRESS);
    std::multimap<IPvXAddress, IPv4Datagram *>::iterator lt = tempdelayPacketlist.getlowerbound(target);
    std::multimap<IPvXAddress, IPv4Datagram *>::iterator ut = tempdelayPacketlist.getupperbound(target);
    for (std::multimap<IPvXAddress, IPv4Datagram *>::iterator it = lt; it != ut; it++) {
        IPv4Datagram *datagram = it->second;
        delayPacketlist.addPacket(target,datagram);
    }
    tempdelayPacketlist.removePacket(target);
}
void RBVTR::sendDataPacket(const IPvXAddress& target,std::vector<std::string> roads,const IPvXAddress nexthop)
{
    EV_LOG("sendDataPacket with nexthop: "+std::string(nexthop.str()));
    RBVTR_EV << "Completing route discovery, originator " << getSelfIPAddress() << ", target " << target <<", nexthop: " <<nexthop<< endl;
    std::cout << "Completing route discovery, originator " << getSelfIPAddress() << ", target " << target <<", nexthop: " <<nexthop<< endl;
    std::multimap<IPvXAddress, IPv4Datagram *>::iterator lt = delayPacketlist.getlowerbound(target);
    std::multimap<IPvXAddress, IPv4Datagram *>::iterator ut = delayPacketlist.getupperbound(target);
    RBVTRPacket * rbvtrdataPacket;
    // reinject the delayed datagrams
    for (std::multimap<IPvXAddress, IPv4Datagram *>::iterator it = lt; it != ut; it++) {
        IPv4Datagram *datagram = it->second;
        if(!dynamic_cast<RBVTRPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))
        {
            if (!dynamic_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket()))
            {
             cPacket * networkPacket = dynamic_cast<cPacket *>(datagram);
             RBVTR_EV << "Sending queued datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress()<<", name: " << networkPacket->getName() << endl;
             std::cout << "Sending queued datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress()<<", name: " << networkPacket->getName() << endl;
             rbvtrdataPacket= createDataPacket(datagram->getDestAddress(), networkPacket->decapsulate());
             rbvtrdataPacket->setroads(roads);
             rbvtrdataPacket->setdesPosition(routingRoad.getPositionTable(datagram->getDestAddress()));
             rbvtrdataPacket->setnexthopAddress(nexthop);
             networkPacket->encapsulate(rbvtrdataPacket);
             BroadcastRTS(rbvtrdataPacket);
             return;
            }else
            {
                EV_LOG("Sending queued rbvtr ");
                rbvtrdataPacket = check_and_cast<RBVTRPacket *>( (dynamic_cast<UDPPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket()))->getEncapsulatedPacket());
                rbvtrdataPacket->nexthop_ip=nexthop;
                RBVTR_EV << "Sending queued rbvtr: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress()<<", name: " << rbvtrdataPacket->getName()<<" nexthop: "<<rbvtrdataPacket->nexthop_ip<< endl;
                std::cout << "Sending queued rbvtr: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress()<<", name: " << rbvtrdataPacket->getName()<<" nexthop: "<<rbvtrdataPacket->nexthop_ip<< endl;
             }
        }else
        {
            EV_LOG("Sending queued rbvtr 2");
            rbvtrdataPacket = check_and_cast<RBVTRPacket *>((dynamic_cast<cPacket *>(datagram))->getEncapsulatedPacket());
            rbvtrdataPacket->nexthop_ip=nexthop;
            RBVTR_EV << "Sending queued rbvtr: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress()<<", name: " << rbvtrdataPacket->getName()<<" nexthop: "<<rbvtrdataPacket->nexthop_ip<< endl;
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
    packetname=packetname+"_"+std::to_string(squmRR++);
    RBVTRPacket * RRPacket = new RBVTRPacket(packetname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    RRPacket->setsrcAddress(getSelfIPAddress());
    RRPacket->setdesAddress(rbvtrPacket->getsrcAddress());
    RRPacket->setdesPosition(rbvtrPacket->getscrPosition());
    EV<<"Create RR : scr: "<<RRPacket->getsrcAddress()<<" des: "<<RRPacket->getdesAddress()<<endl;
    RRPacket->setRBVTRPacketType(RBVTR_RR);
    RRPacket->setSeqnum(squmRR);
    RRPacket->setscrPosition(getSelfPosition());
    RRPacket->setroads(rbvtrPacket->getroads());
    RRPacket->setBitLength(RRPacket->getPacketlength());
    RRPacket->setLifetime(simTime()+RRliftime);
    RRPacket->setcurrentroad(getRoadID());
    //RRPacket->addpassedroad(getRoadID());
   // rBVTRPacket->encapsulate(content);
    return RRPacket;
}
RBVTRPacket *RBVTR::createRTSPacket(RBVTRPacket *rbvtrPacket)
{
    std::string packetname=rbvtrPacket->getName();
  //  std::string packettype="RD_";
    //packetname=packetname.replace(packetname.begin(),packetname.begin()+2,"RTS");
    simtime_t lifetime;
    if(packetname.find("RTS_")==std::string::npos){
    packetname="RTS_"+packetname;
    squmRTS++;
    lifetime=simTime()+RRliftime;
    }else
    {
        lifetime=rbvtrPacket->getLifetime();
    }
    RBVTRPacket * RTSPacket = new RBVTRPacket(packetname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    RTSPacket->setsrcAddress(getSelfIPAddress());
    RTSPacket->setdesAddress(rbvtrPacket->getdesAddress());
    RTSPacket->setdesPosition(rbvtrPacket->getdesPosition());
    EV<<"Create RTS : scr: "<<RTSPacket->getsrcAddress()<<" des: "<<RTSPacket->getdesAddress()<<endl;
    cout<<"Create RTS : scr: "<<RTSPacket->getsrcAddress()<<" des: "<<RTSPacket->getdesAddress()<<endl;
    RTSPacket->setRBVTRPacketType(RBVTR_RTS);
    RTSPacket->setSeqnum(squmRTS);
    RTSPacket->setscrPosition(getSelfPosition());
    RTSPacket->setroads(rbvtrPacket->getroads());
    RTSPacket->setBitLength(RTSPacket->getPacketlength());
    RTSPacket->setLifetime(lifetime);
    RTSPacket->setpassedroads(rbvtrPacket->getpassedroads());
    RTSPacket->setsenderPosition(getSelfPosition());
    return RTSPacket;
}
RBVTRPacket *RBVTR::createREPacket(RBVTRPacket *rbvtrPacket)
{
    std::string packetname=std::string("RE_")+rbvtrPacket->getName();
  //  std::string packettype="RD_";
    //packetname=packetname.replace(packetname.begin(),packetname.begin()+2,"RTS");
    simtime_t lifetime;
    lifetime=simTime()+RRliftime;
    RBVTRPacket * REPacket = new RBVTRPacket(packetname.c_str());
    //std::cout <<"rBVTRPacket-getName = " << RBVTRPacket(strcat("RD_",content->getName())) << endl;
    REPacket->setsrcAddress(rbvtrPacket->getdesAddress());
    REPacket->setdesAddress(rbvtrPacket->getsrcAddress());
    REPacket->setdesPosition(rbvtrPacket->getscrPosition());
    EV<<"Create RTS : scr: "<<REPacket->getsrcAddress()<<" des: "<<REPacket->getdesAddress()<<endl;
    cout<<"Create RTS : scr: "<<REPacket->getsrcAddress()<<" des: "<<REPacket->getdesAddress()<<endl;
    REPacket->setRBVTRPacketType(RBVTR_RE);
    REPacket->setSeqnum(squmRE++);
    REPacket->setscrPosition(getSelfPosition());
    REPacket->setroads(rbvtrPacket->getroads());
    REPacket->setBitLength(rbvtrPacket->getPacketlength());
    REPacket->setLifetime(lifetime);
    vector <string> passedroad;
    for (int i=0;i<rbvtrPacket->getroads().size();i++)
    {
        if(std::find(rbvtrPacket->getpassedroads().begin(),rbvtrPacket->getpassedroads().end(),rbvtrPacket->getroads()[i])==rbvtrPacket->getpassedroads().end())
        {
            passedroad.push_back(rbvtrPacket->getroads()[i]);
        }
    }
    REPacket->setpassedroads(passedroad);
    return REPacket;
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
    CTSPacket->setsenderPosition(rbvtrPacket->getsenderPosition());

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
void RBVTR::EV_LOG(std::string context)
{
    RouteInterface::EV_LOG("RBVTR",context);
}
