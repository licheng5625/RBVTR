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

#include "DelayPacketTable.h"





void DelayPacketTable::addPacket(const IPvXAddress & address,IPv4Datagram * packet)
{

 delaytable.insert( std::pair<IPvXAddress, IPv4Datagram *>(address, packet));

}


bool DelayPacketTable::hasPacket(const IPvXAddress & address)   {

    AddressToPacket::const_iterator it = delaytable.find(address);
              if (it == delaytable.end())
              {
                  return true;
              }
              else
              {
                  return false;
              }
}



void DelayPacketTable::removePacket(const IPvXAddress & address) {
        std::multimap<IPvXAddress, IPv4Datagram *>::iterator lt = getlowbound(address);
        std::multimap<IPvXAddress, IPv4Datagram *>::iterator ut = getupperbound(address);
        delaytable.erase(lt, ut);
}



std::multimap<IPvXAddress, IPv4Datagram *>::iterator DelayPacketTable::getlowbound(const IPvXAddress& target)
        {
            return delaytable.lower_bound(target);
        }

std::multimap<IPvXAddress, IPv4Datagram *>::iterator DelayPacketTable::getupperbound(const IPvXAddress& target)
      {
          return delaytable.upper_bound(target);
      }


void DelayPacketTable::clear() {
    delaytable.clear();
 }


