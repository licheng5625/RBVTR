//
// Copyright (C) 2004 Andras Varga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#include "BroadcastWaitingTable.h"





void PacketWaitingTable::addPacket(cMessage * timer ,RBVTRPacket * packet,IPv4Datagram * ipv4)
{
    waitinglist[timer] = RBVTRPacketTOIPv4Datagram(packet, ipv4);

    // waitinglist[timer] = packet;
}

RBVTRPacket * PacketWaitingTable::getRBVTRPacket(cMessage * timer )
{
    TimetoRDPacket::const_iterator it = waitinglist.find(timer);
        if (it == waitinglist.end())
            return NULL;
        else
        {
            return it->second.first;
        }
}

IPv4Datagram * PacketWaitingTable::getDataPacket(cMessage * timer )
{
    TimetoRDPacket::const_iterator it = waitinglist.find(timer);
        if (it == waitinglist.end())
            return NULL;
        else
        {
            return it->second.second;
        }
}

void PacketWaitingTable::removePacket(cMessage*  timer) {
    TimetoRDPacket::iterator it = waitinglist.find(timer);
    //delete it->second;
    waitinglist.erase(it);
   // cancelAndDelete(timer);
}

cMessage* PacketWaitingTable::findPacket(RBVTRPacket*  packet) {
    for (TimetoRDPacket::const_iterator it = waitinglist.begin(); it != waitinglist.end(); it++)
    {
        if(it->second.first->getSeqnum()==packet->getSeqnum()&&it->second.first->getdesAddress()==packet->getdesAddress()&&it->second.first->getPacketType()==packet->getPacketType())
        {
            return it->first;
        }
    }
    return NULL;
}

void PacketWaitingTable::clear() {
    waitinglist.clear();
}


