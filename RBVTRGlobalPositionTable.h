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

#ifndef __INET_RBVTRGLOBLEPOSITIONTABLE_H_
#define __INET_RBVTRGLOBLEPOSITIONTABLE_H_

#include <vector>
#include <map>
#include "INETDefs.h"
#include "IPvXAddress.h"
#include "Coord.h"
#include "../routeInterface/GlobalPositionTable.h"
/**
 * This class provides a mapping between node addresses and their positions.
 */
class INET_API RBVTRGlobalPositionTable:  public GlobalPositionTable {
    private:
         typedef std::map<IPvXAddress, std::string> AddressToRoad;
         AddressToRoad addressToRoad;

    public:
         RBVTRGlobalPositionTable() { }

        std::string getAddressToRoad(IPvXAddress address, std::string raod);

        void setAddressToRoad(IPvXAddress address, std::string raod);

        void removePosition(const IPvXAddress & address);

        void clear();

};

class intersection{
public:
        typedef std::pair<std::string, std::string> intersectionofroad;
        intersection(std::string scr,std::string des)
        {
            intersectionofroad(scr,des);
        }
};
#endif
