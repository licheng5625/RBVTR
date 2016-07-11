//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package inet.networklayer.routing.RBVTR;
  
//moduleinterface IRBVTR
//{
//    parameters:
//        @display("i=block/routing");
////        string routingTableModule = default("^.routingTable");
//        
//    gates:
//        input ipIn;
//        output ipOut;
//}
       
simple RBVTR  
{
    parameters:
        @display("i=block/routing");
      string networkProtocolModule = default("^.networkLayer.ip");
       string routingTableModule = default("^.routingTable");
      
        string interfaces = default("*");
		double RDliftime @unit("s") = default(20s); 
	    double RUliftime @unit("s") = default(20s); 
		double RRliftime @unit("s") = default(20s); 
		double REwaittime @unit("s") = default(0.5s); 
		double DATAliftime @unit("s") = default(20s); 
		double nextRUtimer @unit("s") = default(0.02s); 
		double Tmax  @unit("s") = default(0.1s);
    	double dopt @unit("m") = default(300m); 
   		double dmax @unit("m") = default(400m); 
    	double a1 =default(0.07);
    	double a2 =default(0.5);
    	double a3 =default(0.03);
    	int section =default(1);
    	int maxREtimes=default(1);
    	bool checkRD=default(true);
    gates:
        input ipIn;
        output ipOut;
}
