
#To find average travel time by 
#!/usr/bin/env python
"""
@file    No-Routing-simulation.py
@author  Shubajit Saha
Date: 20-11-2016
"""


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
from IPython.display import display, HTML

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    ## Set __file__ as current file name
    #__file__ = "/home/shuvojit/Project/Simulation/DynamicRouteSuggestionSystem/docs/tutorial/traci_tls/Simulation-Notebook.ipnb"
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
#Generates Route file for example graph


def generate_routefile():
    N = 100  # number of initial vehicles
    # demand per second from different directions
    vehNr = 0
    with open("sim.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="vtype0" accel="0.8" decel="4.5" sigma="0.5" length="2" minGap="1" maxSpeed="16.67" guiShape="passenger" reroute="false"/>
        <vType id="vtype1" accel="0.8" decel="4.5" sigma="0.5" length="2" minGap="1" maxSpeed="16.67" guiShape="passenger"/>
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
        
        
        count = [[0 for j in range(3)] for i in range(3)]
        for i in range(N):
            for j in range(100):
                src = random.randint(1,3)
                dest = random.randint(8,10)
                rid = "r"+str(src)+"to"+str(dest)
                print('    <vehicle id="vid#%i#%i#%i" type="vtype1" route="%s" depart="%i" />' % (
                    vehNr,src,dest,rid, i), file=routes)
                vehNr += 1
                count[src-1][dest-8] = count[src-1][dest-8]+1; #input specific

        print("Vehicles Generated: ",pd.DataFrame(count))
        print("</routes>", file=routes)


#Route Choice Server
def run():
    """execute the TraCI control loop"""
    print("Control loop")
    step = 0
    vehNr = 10000
    edges = traci.edge.getIDList();
    print("num edgees = %i",traci.edge.getIDCount())
 
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep() 
        #print("running step no. %i",step)
        if step <= 100:            
            print("Dynamic Vehicles: ==================================")
            for x in range(100):
                src = random.randint(1,3)
                dest = random.randint(8,10)
                vid = "vid#"+str(vehNr)+'#'+str(src)+'#'+str(dest)
                rid = "r"+str(src)+"to"+str(dest)
                traci.vehicle.add(vid,rid,typeID="vtype1",depart = step,lane = 0)
                vehNr += 1
            step += 1
    return



def processData():
    xmldoc = minidom.parse('simtripinfo.xml')
    itemlist = xmldoc.getElementsByTagName('tripinfo')
    departlist = []
    durationlist = []
    idlist = []
    for s in itemlist:
        departlist.append(float(s.attributes['depart'].value))
        durationlist.append(float(s.attributes['duration'].value))
        t = s.attributes['id'].value
        t = t.split('#')
        src = int(t[2])
        des = int(t[3])
        route_id = (src-1)*3+(des-8)
        idlist.append(route_id)
 
    print(len(durationlist)," ",len(departlist))
    df = pd.DataFrame({'depart_time':departlist,'duration':durationlist,'id':idlist})
    print("Average Travel Duration: ",df['duration'].mean())



if __name__ == "__main__":
    random.seed(42)  # make tests reproducible
    sumoBinary = checkBinary('sumo-gui')
    file = open("log.txt",'w')
    sys.stdout = file

    #genrate route file
    generate_routefile()
    
    #Start Simulation
    traci.start([sumoBinary, "-c", "sim.sumocfg",
                         "--tripinfo-output", "simtripinfo.xml","--device.rerouting.probability","0.00","--summary","summary.xml"])
    
    
    #run Route Choice Server
    run()
    print("simulation completed successfully!!")
    
    #Stop simulation
    traci.close()
    sys.stdout.flush()
    print("simulation completed successfully!!")

    #Process Data
    print("Processing Data: ")
    processData()

