from __future__ import absolute_import
from __future__ import print_function

import logging
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
#================ SUMO SETUP CODE ========================================
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

# def generate_routefile():
#     
#     N = Parameters.iterations  # number of time steps
#     # demand per second from different directions
    
#     with open("testdata/sim.rou.xml", "w") as routes:
#         print("""<routes>
#         <vType id="vtype0" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
#         <vType id="vtype1" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
#         <route id="r1to2" edges="1to2 " />
#         <route id="r1to3" edges="1to3 " />
#         <route id="r1to4" edges="1to4 " />
#         <route id="r1to5" edges="1to4 4to5 " />
#         <route id="r1to6" edges="1to4 4to6 " />
#         <route id="r1to7" edges="1to4 4to7 " />
#         <route id="r1to8" edges="1to4 4to7 7to8 " />
#         <route id="r1to9" edges="1to4 4to7 7to9 " />
#         <route id="r1to10" edges="1to4 4to7 7to10 " />
#         <route id="r2to3" edges="2to3 " />
#         <route id="r2to4" edges="2to4 " />
#         <route id="r2to5" edges="2to5 " />
#         <route id="r2to6" edges="2to5 5to6 " />
#         <route id="r2to7" edges="2to5 5to7 " />
#         <route id="r2to8" edges="2to5 5to8 " />
#         <route id="r2to9" edges="2to5 5to8 8to9 " />
#         <route id="r2to10" edges="2to5 5to8 8to10 " />
#         <route id="r3to4" edges="3to4 " />
#         <route id="r3to5" edges="3to5 " />
#         <route id="r3to6" edges="3to6 " />
#         <route id="r3to7" edges="3to6 6to7 " />
#         <route id="r3to8" edges="3to6 6to8 " />
#         <route id="r3to9" edges="3to6 6to9 " />
#         <route id="r3to10" edges="3to6 6to9 9to10 " />
#         <route id="r4to5" edges="4to5 " />
#         <route id="r4to6" edges="4to6 " />
#         <route id="r4to7" edges="4to7 " />
#         <route id="r4to8" edges="4to7 7to8 " />
#         <route id="r4to9" edges="4to7 7to9 " />
#         <route id="r4to10" edges="4to7 7to10 " />
#         <route id="r5to6" edges="5to6 " />
#         <route id="r5to7" edges="5to7 " />
#         <route id="r5to8" edges="5to8 " />
#         <route id="r5to9" edges="5to8 8to9 " />
#         <route id="r5to10" edges="5to8 8to10 " />
#         <route id="r6to7" edges="6to7 " />
#         <route id="r6to8" edges="6to8 " />
#         <route id="r6to9" edges="6to9 " />
#         <route id="r6to10" edges="6to9 9to10 " />
#         <route id="r7to8" edges="7to8 " />
#         <route id="r7to9" edges="7to9 " />
#         <route id="r7to10" edges="7to10 " />
#         <route id="r8to9" edges="8to9 " />
#         <route id="r8to10" edges="8to10 " />
#         <route id="r9to10" edges="9to10 " />""", file=routes)
#         lastVeh = 0
#         vehNr = 0
#         count = [[0 for j in range(3)] for i in range(3)]
#         for i in range(N):
#             src = random.randint(1,3)
#             dest = random.randint(8,10)
#             rid = "r"+str(src)+"to"+str(dest)
#             print('    <vehicle id="vid#%i#%i#%i" type="vtype1" route="%s" depart="%i" />' % (
#                 vehNr,src,dest,rid, i), file=routes)
#             vehNr += 1
#             lastVeh = i
#             count[src-1][dest-8] = count[src-1][dest-8]+1; #input specific

#         #print("Vehicles Generated: ",pd.DataFrame(count))
#         print("</routes>", file=routes)


def constructGraph(nodeFile,edgeFile):
    xmldoc = minidom.parse(nodeFile)
    itemlist = xmldoc.getElementsByTagName('node')
    nodemap = dict()
    for i,s in enumerate(itemlist):
        t = s.attributes['id'].value
        nodemap[t] = i
    #print(nodemap)

    xmldoc = minidom.parse(edgeFile)
    itemlist = xmldoc.getElementsByTagName('edge')
    edgemap = dict()
    for i,s in enumerate(itemlist):
        t = s.attributes['id'].value
        edgemap[t] = i
    print(edgemap)

    lanelist = traci.lane.getIDList()
    distmap = dict()
    for lane in lanelist:
        edgeid = traci.lane.getEdgeID(lane)
        if edgemap.has_key(edgeid):
            print("edgeid: ",edgeid," id: ",edgemap[edgeid],"Length: ",traci.lane.getLength(lane))
            distmap[edgeid] = traci.lane.getLength(lane)

    N = len(nodemap)
    print(N)
    graph = [[0 for j in range(N)]for i in range(N)]
    for i,s in enumerate(itemlist):
        src = s.attributes['from'].value
        des = s.attributes['to'].value
        if distmap.has_key(s):
            print(str(nodemap[src])+" to "+str(nodemap[des])+"dist: "+str(distmap[s]))
            graph[nodemap[src]][nodemap[des]] = distmap[s]
        
    return graph,nodemap,edgemap

def setupLogging(filename,level = logging.DEBUG):
    logging.basicConfig(filename=filename,level=level)
    return

def setupSumo(configFile,outputFile):
    sumoBinary = checkBinary('sumo-gui')
    traci.start([sumoBinary, "-c",configFile,"--tripinfo-output",outputFile])
    return

def init(logFile,configFile,outputFile,routeFile=""):
    setupLogging(logFile)
    #generate_routefile(routeFile)
    setupSumo(configFile,outputFile)
    return

if __name__ == "__main__":

    #Start Simulation
    init('log',"bangalore.sumocfg","Bangalore-simtripinfo.xml")
    

    #genrate route file
    E = traci.edge.getIDCount();
    edges = traci.edge.getIDList();
    print("num edges = %i",traci.edge.getIDCount())
    #print("edge list",edges)

    print("num Nodes = %i",traci.junction.getIDCount())
    N = traci.junction.getIDCount() #no. of junctions
    nodes = traci.junction.getIDList();
    #print("Node list",nodes)


    graph,nodemap,edgemap = constructGraph('bangalore.nod.xml','bangalore.edg.xml')
    #print(graph)
   # dist,qvalue = preProcess(nodes,edges)



    
        
