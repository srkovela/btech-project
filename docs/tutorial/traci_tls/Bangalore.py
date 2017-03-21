from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
from Queue import PriorityQueue
from xml.dom import minidom
import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
# we need to import python modules from the $SUMO_HOME/tools directory

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci

#============================================================================================================

def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = Parameters.iterations  # number of time steps
    # demand per second from different directions
    
    with open("testdata/sim.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="vtype0" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="vtype1" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <route id="r1to2" edges="1to2 " />
        <route id="r1to3" edges="1to3 " />
        <route id="r1to4" edges="1to4 " />
        <route id="r1to5" edges="1to4 4to5 " />
        <route id="r1to6" edges="1to4 4to6 " />
        <route id="r1to7" edges="1to4 4to7 " />
        <route id="r1to8" edges="1to4 4to7 7to8 " />
        <route id="r1to9" edges="1to4 4to7 7to9 " />
        <route id="r1to10" edges="1to4 4to7 7to10 " />
        <route id="r2to3" edges="2to3 " />
        <route id="r2to4" edges="2to4 " />
        <route id="r2to5" edges="2to5 " />
        <route id="r2to6" edges="2to5 5to6 " />
        <route id="r2to7" edges="2to5 5to7 " />
        <route id="r2to8" edges="2to5 5to8 " />
        <route id="r2to9" edges="2to5 5to8 8to9 " />
        <route id="r2to10" edges="2to5 5to8 8to10 " />
        <route id="r3to4" edges="3to4 " />
        <route id="r3to5" edges="3to5 " />
        <route id="r3to6" edges="3to6 " />
        <route id="r3to7" edges="3to6 6to7 " />
        <route id="r3to8" edges="3to6 6to8 " />
        <route id="r3to9" edges="3to6 6to9 " />
        <route id="r3to10" edges="3to6 6to9 9to10 " />
        <route id="r4to5" edges="4to5 " />
        <route id="r4to6" edges="4to6 " />
        <route id="r4to7" edges="4to7 " />
        <route id="r4to8" edges="4to7 7to8 " />
        <route id="r4to9" edges="4to7 7to9 " />
        <route id="r4to10" edges="4to7 7to10 " />
        <route id="r5to6" edges="5to6 " />
        <route id="r5to7" edges="5to7 " />
        <route id="r5to8" edges="5to8 " />
        <route id="r5to9" edges="5to8 8to9 " />
        <route id="r5to10" edges="5to8 8to10 " />
        <route id="r6to7" edges="6to7 " />
        <route id="r6to8" edges="6to8 " />
        <route id="r6to9" edges="6to9 " />
        <route id="r6to10" edges="6to9 9to10 " />
        <route id="r7to8" edges="7to8 " />
        <route id="r7to9" edges="7to9 " />
        <route id="r7to10" edges="7to10 " />
        <route id="r8to9" edges="8to9 " />
        <route id="r8to10" edges="8to10 " />
        <route id="r9to10" edges="9to10 " />""", file=routes)
        lastVeh = 0
        vehNr = 0
        count = [[0 for j in range(3)] for i in range(3)]
        for i in range(N):
            src = random.randint(1,3)
            dest = random.randint(8,10)
            rid = "r"+str(src)+"to"+str(dest)
            print('    <vehicle id="vid#%i#%i#%i" type="vtype1" route="%s" depart="%i" />' % (
                vehNr,src,dest,rid, i), file=routes)
            vehNr += 1
            lastVeh = i
            count[src-1][dest-8] = count[src-1][dest-8]+1; #input specific

        #print("Vehicles Generated: ",pd.DataFrame(count))
        print("</routes>", file=routes)




if __name__ == "__main__":
        sumoBinary = checkBinary('sumo-gui')
        file = open("log.txt",'w')
        sys.stdout = file

        #genrate route file
        #generate_routefile()
        
        #Start Simulation
        traci.start([sumoBinary, "-c", "Bangalore-Simulation/bangalore.sumocfg",
                             "--tripinfo-output", "simtripinfo.xml"])
        
        
#         g=[[1,2,3],[2,3,4],[3,4,5],[4,5,6],[5,6,7],[6,7,8],[7,8,9],[8,9],[9],[]]
#         #change weight test
#         w = [[10, 10, 1], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10], [10], []]      
#         N = len(g)
#         dist,qf = preProcess(g,w)
# #         display(pd.DataFrame(dist))
# #         display(pd.DataFrame(qf))
        

#         print("Print Q-Values Initial:=============== \n",qf[0][8],"\n===================")
#         print("Print Q-Values Initial:=============== \n",qf[2][8],"\n===================")

#         run(w,dist,qf,g)
#         print("simulation completed successfully!!")
        
#         traci.close()
#         sys.stdout.flush()
#         print("simulation completed successfully!!")

#         display(qf)
#         #processData()