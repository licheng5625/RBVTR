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

#include "SeenTable.h"





void SeenTable::SeePacket(const IPvXAddress & address, int squm)
{
    ASSERT(!address.isUnspecified());
    AddressToSqum::const_iterator it = seenlist.find(address);
    if (it == seenlist.end())
    {
        std::set<int> templist;
        seenlist[address]=templist;
    }
    seenlist[address].insert(squm);
}

bool SeenTable::isSeenPacket(const IPvXAddress & address,int squm)   {
    AddressToSqum::const_iterator it = seenlist.find(address);
        if (it == seenlist.end())
            {
            SeePacket( address,  squm) ;
            return false;
            }

        else
        {
            std::set<int> templist =it->second;

            if(templist.find(squm)==templist.end())
             {
                 SeePacket( address,squm) ;
                 return false;
             }
        }
        return true;
}

void SeenTable::removeSeenPacket(const IPvXAddress & address) {
    AddressToSqum::iterator it = seenlist.find(address);
    seenlist.erase(it);
}



void SeenTable::clear() {
    seenlist.clear();
}


