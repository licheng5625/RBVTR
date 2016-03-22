/*
 * RBVTRPacket.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: chengli
 */

#include <RBVTR/RBVTRPacket.h>


#include <omnetpp.h>



 #include "IPvXAddress.h"
#include "simtime_t.h"



template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}
Register_Class(RBVTRPacket);

RBVTRPacket::RBVTRPacket(const char *name, int kind) : ::cPacket(name,kind)
{   roads=std::vector<std::string>();
    passedroads=std::vector<std::string>();

    // TODO Auto-generated constructor stub

}
RBVTRPacket::RBVTRPacket(const RBVTRPacket& other) : ::cPacket(other)
{
    copy(other);
}
simtime_t & RBVTRPacket::getLifetime()
{
    return lifeTime;
}

  void  RBVTRPacket::setLifetime(simtime_t mylifetime)
  {
     lifeTime =mylifetime;
  }

RBVTRPacket::~RBVTRPacket() {
    // TODO Auto-generated destructor stub
}
RBVTRPacket& RBVTRPacket::operator=(const RBVTRPacket& other)
{
    if (this==&other) return *this;
    ::cPacket::operator=(other);
    copy(other);
    return *this;
}

void RBVTRPacket::copy(const RBVTRPacket& other)
{
     this->packetTpye_var= other.packetTpye_var;
     this->src_ip= other.src_ip;
     this->des_ip= other.des_ip;
     this->nexthop_ip= other.nexthop_ip;
     this->src_position= other.src_position;
     this->sender_position= other.sender_position;
     this->des_position= other.des_position;
     this->seqNum= other.seqNum;
     this->roads= other.roads;
     this->passedroads= other.passedroads;
     this->lifeTime=other.lifeTime;
}

void RBVTRPacket::parsimPack(cCommBuffer *b)
{
    ::cPacket::parsimPack(b);
    doPacking(b,this->packetTpye_var);
    doPacking(b,this->src_ip);
    doPacking(b,this->des_ip);
    doPacking(b,this->nexthop_ip);
    doPacking(b,this->src_position);
    doPacking(b,this->sender_position);
    doPacking(b,this->des_position);
    doPacking(b,this->seqNum);
    doPacking(b,this->roads);
    doPacking(b,this->passedroads);
    doPacking(b,this->lifeTime);
}

void RBVTRPacket::parsimUnpack(cCommBuffer *b)
{
    ::cPacket::parsimUnpack(b);
    doUnpacking(b,this->packetTpye_var);
    doUnpacking(b,this->src_ip);
    doUnpacking(b,this->des_ip);
    doUnpacking(b,this->nexthop_ip);
    doUnpacking(b,this->src_position);
    doUnpacking(b,this->des_position);
    doUnpacking(b,this->sender_position);
    doUnpacking(b,this->seqNum);
    doUnpacking(b,this->roads);
    doUnpacking(b,this->passedroads);
    doUnpacking(b,this->lifeTime);
}

IPvXAddress& RBVTRPacket::getsrcAddress()
{
    return src_ip;
}

void RBVTRPacket::setsrcAddress(const IPvXAddress& address)
{
    this->src_ip = address;
}

IPvXAddress& RBVTRPacket::getdesAddress()
{
    return des_ip;
}

void RBVTRPacket::setdesAddress(const IPvXAddress& address)
{
    this->des_ip = address;
}

IPvXAddress& RBVTRPacket::getnexthopAddress()
{
    return nexthop_ip;
}

void RBVTRPacket::setnexthopAddress(const IPvXAddress& address)
{
    this->nexthop_ip = address;
}
Coord& RBVTRPacket::getscrPosition()
{
    return src_position;
}
Coord& RBVTRPacket::getdesPosition()
{
    return des_position;
}



Coord& RBVTRPacket::getsenderPosition()
{
    return sender_position;
}

void RBVTRPacket::setsenderPosition(const Coord& address)
{
    this->sender_position = address;
}
int RBVTRPacket::getPacketlength()
{
    // routingMode
    int routingMode = 1;
    // destinationPosition, perimeterRoutingStartPosition, perimeterRoutingForwardPosition
    int positions = 8 * 3 * 2 * 4;
    // currentFaceFirstSenderAddress, currentFaceFirstReceiverAddress, senderAddress
    int addresses = 8 * 3 * 4 * 3;
    // TODO: address size
    int squm=8;
    int road=roads.size()*8*2;
    int time =8;
    return routingMode + positions +  addresses+squm+road+time;
 }

void RBVTRPacket::setSeqnum(const unsigned int& seqNum)
{
    this->seqNum = seqNum;
}
unsigned int& RBVTRPacket::getSeqnum()
{
    return seqNum;
}

void RBVTRPacket::setroads(std::vector<std::string> myroads)
{
    roads=myroads;
}

std::vector<std::string>& RBVTRPacket::getroads(){
    return roads;
}
void RBVTRPacket::addroad(std::string road){
    return roads.push_back(road);
}
void RBVTRPacket::setpassedroads(std::vector<std::string> myroads)
{
    passedroads=myroads;
}

std::vector<std::string>& RBVTRPacket::getpassedroads(){
    return passedroads;
}
void RBVTRPacket::addpassedroad(std::string road){
    return passedroads.push_back(road);
}
std::string& RBVTRPacket::getroadsToStr(){
    std::string ret="";
    for(int i=0;i<roads.size();i++)
    {
        ret+=roads[i];
    }
    return ret;
}

void RBVTRPacket::setscrPosition(const Coord& position)
{
    this->src_position = position;
}
void RBVTRPacket::setdesPosition(const Coord& position)
{
    this->des_position = position;
}
class RBVTRPacketDescriptor : public cClassDescriptor
{
  public:
    RBVTRPacketDescriptor();
    virtual ~RBVTRPacketDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(RBVTRPacketDescriptor);

RBVTRPacketDescriptor::RBVTRPacketDescriptor() : cClassDescriptor("RBVTRPacket", "cPacket")
{
}

RBVTRPacketDescriptor::~RBVTRPacketDescriptor()
{
}

bool RBVTRPacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<RBVTRPacket *>(obj)!=NULL;
}

const char *RBVTRPacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int RBVTRPacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 2+basedesc->getFieldCount(object) : 7;
}

unsigned int RBVTRPacketDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
            FD_ISEDITABLE,
            FD_ISEDITABLE,
            FD_ISEDITABLE,
            FD_ISEDITABLE,
            FD_ISEDITABLE,
            FD_ISEDITABLE,
            FD_ISEDITABLE,
            FD_ISEDITABLE,
    };
    return (field>=0 && field<2) ? fieldTypeFlags[field] : 0;
}

const char *RBVTRPacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "packetTpye_var",
        "src_ip",
        "des_ip",
        "nexthop_ip",
        "src_position",
        "des_position",
        "seqNum",
        "roads",
        "lifetime",
    };
    return (field>=0 && field<2) ? fieldNames[field] : NULL;
}

int RBVTRPacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='p' && strcmp(fieldName, "packetTpye_var")==0) return base+0;
    if (fieldName[0]=='s' && strcmp(fieldName, "src_ip")==0) return base+1;
    if (fieldName[0]=='d' && strcmp(fieldName, "des_ip")==0) return base+2;
    if (fieldName[0]=='n' && strcmp(fieldName, "nexthop_ip")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "src_position")==0) return base+4;
    if (fieldName[0]=='d' && strcmp(fieldName, "des_position")==0) return base+5;
    if (fieldName[0]=='s' && strcmp(fieldName, "seqNum")==0) return base+6;
    if (fieldName[0]=='r' && strcmp(fieldName, "roads")==0) return base+7;
    if (fieldName[0]=='l' && strcmp(fieldName, "lifetime")==0) return base+8;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *RBVTRPacketDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {

            "RBVTRPacketType",
            "IPvXAddress",
            "IPvXAddress",
            "IPvXAddress",
            "Coord",
            "Coord",
            "unsigned int"  ,
            "std::vector<std::string>"  ,
            "simtime_t",
    };
    return (field>=0 && field<2) ? fieldTypeStrings[field] : NULL;
}

const char *RBVTRPacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    }
}

int RBVTRPacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    RBVTRPacket *pp = (RBVTRPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string RBVTRPacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    RBVTRPacket *pp = (RBVTRPacket *)object; (void)pp;
    switch (field) {
        case 0: {std::stringstream out; out << pp->getPacketType(); return out.str();}
        case 1: {std::stringstream out; out << pp->getsrcAddress(); return out.str();}
        case 2: {std::stringstream out; out << pp->getdesAddress(); return out.str();}
        case 3: {std::stringstream out; out << pp->getnexthopAddress(); return out.str();}
        case 4: {std::stringstream out; out << pp->getscrPosition(); return out.str();}
        case 5: {std::stringstream out; out << pp->getdesPosition(); return out.str();}
        case 6: {std::stringstream out; out << pp->getSeqnum(); return out.str();}
        case 7: {std::stringstream out; out << pp->getroadsToStr(); return out.str();}
        case 8: {std::stringstream out; out << pp->getLifetime(); return out.str();}
        default: return "";
    }
}

bool RBVTRPacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    RBVTRPacket *pp = (RBVTRPacket *)object; (void)pp;
    switch (field) {
        default: return false;
    }
}



const char *RBVTRPacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
   switch (field) {
            case 0: return opp_typename(typeid(RBVTRPacketType));
            case 1: return opp_typename(typeid(IPvXAddress));
            case 2: return opp_typename(typeid(IPvXAddress));
            case 3: return opp_typename(typeid(IPvXAddress));
            case 4: return opp_typename(typeid(Coord));
            case 5: return opp_typename(typeid(Coord));
            case 6: return opp_typename(typeid(unsigned int));
            case 7: return opp_typename(typeid(std::vector<std::string>));
            case 8: return opp_typename(typeid(simtime_t));
        default: return NULL;
    };
}

void *RBVTRPacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    RBVTRPacket *pp = (RBVTRPacket *)object; (void)pp;
    switch (field) {
        case 0: return (void *)(&pp->getPacketType()); break;
        case 1: return (void *)(&pp->getsrcAddress()); break;
        case 2: return (void *)(&pp->getdesAddress()); break;
        case 3: return (void *)(&pp->getnexthopAddress()); break;
        case 4: return (void *)(&pp->getscrPosition()); break;
        case 5: return (void *)(&pp->getdesPosition()); break;
        case 6: return (void *)(&pp->getSeqnum()); break;
        case 7: return (void *)(&pp->getroads()); break;
        case 8: return (void *)(&pp->getLifetime()); break;
        default: return NULL;
    }
}
