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

#include "RTSPacketTable.h"





void RTSPacketTable::addPacket(int squm ,RBVTRPacket * packet,IPv4Datagram * ipv4)
{
    waitinglist[squm] = RBVTRPacketTOIPv4Datagram(packet, ipv4);

    // waitinglist[squm] = packet;
}

RBVTRPacket * RTSPacketTable::getRBVTRPacket(int squm )
{
    SqumtoRDPacket::const_iterator it = waitinglist.find(squm);
        if (it == waitinglist.end())
            return NULL;
        else
        {
            return it->second.first;
        }
}
void  RTSPacketTable::addNexthop(int squm ,IPv4Address ipv4)
{
    nexthoplist[squm] =  ipv4;
}

bool RTSPacketTable::isnexthop(int squm)
{
    SqumtoNextHop::const_iterator it = nexthoplist.find(squm);
               if (it == nexthoplist.end())
                   return false;
               else
               {
                   return true;
               }
}
IPv4Address RTSPacketTable::findnexthop(int squm)
{
    SqumtoNextHop::const_iterator it = nexthoplist.find(squm);
    return it->second;

}
IPv4Datagram * RTSPacketTable::getDataPacket(int squm )
{
    SqumtoRDPacket::const_iterator it = waitinglist.find(squm);
        if (it == waitinglist.end())
            return NULL;
        else
        {
            return it->second.second;
        }
}

void RTSPacketTable::removePacket(int squm) {
    SqumtoRDPacket::iterator it = waitinglist.find(squm);
    //delete it->second;
    waitinglist.erase(it);
   // cancelAndDelete(squm);
}

int  RTSPacketTable::findPacket(RBVTRPacket*  packet) {
    for (SqumtoRDPacket::const_iterator it = waitinglist.begin(); it != waitinglist.end(); it++)
    {
        if(it->second.first->getSeqnum()==packet->getSeqnum()&&it->second.first->getsrcAddress()==packet->getsrcAddress())
        {
            return it->first;
        }
    }
    return NULL;
}

void RTSPacketTable::clear() {
    waitinglist.clear();
}


