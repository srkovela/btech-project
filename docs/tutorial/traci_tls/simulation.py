#!/usr/bin/env python
"""
@file    simulation.py
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
from pandas import *

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

class Parameters:
    inf = 1e8
    delta = 1.5 #mindist multiplier
    beta = 0.5 #min travel tims vs min path trade off
    alpha = 0.1 #learning rate
    epsilon =  0.4 #exploartion vs exploitation
    gamma = 0.1 #discount factor
    cona = 1 #constant multiplier for congestion
    conb = 2 #length increases quadratically to the congestion


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
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
            print('    <vehicle id="vid_%i" type="vtype1" route="%s" depart="%i" />' % (
                vehNr,rid, i), file=routes)
            vehNr += 1
            lastVeh = i
            count[src-1][dest-8] = count[src-1][dest-8]+1; #input specific

        print("Vehicles Generated: ",DataFrame(count))
        print("</routes>", file=routes)


'''
shortest distance sdist[][] 2-d array of shortest distance from each path
qvalues[][] 2-d array of qvalues for each junction
'''

def preProcess(edgeweights,sdist,qvalues):
    N = traci.junction.getIDCount() #no. of junctions
    '''write djikstra'''
    print("no. of nodes: ",N)
    pq = PriorityQueue()
    for i in range(N):
        pq.put((0,i))
        sdist[i][i] = 0
        while pq.qsize() > 0:
            top = pq.get()
            print("top: ",top)
            u = top[1]
            for j in range(N):
                if  edgeweights[u][j] > 0 and sdist[i][u]+edgeweights[u][j] < sdist[i][j]:
                    sdist[i][j] = sdist[i][u] + edgeweights[u][j]
                    pq.put((sdist[i][j],j))
    #print(sdist)
    ''' initilaise q values with djikstra shortest paths'''
    for i in range(N):
        for j in range(N):
            for k in range(N):
                if edgeweights[i][k] > 0 and sdist[k][j] < Parameters.inf:
                    qvalues[i][j][k] = -1*(sdist[k][j]+edgeweights[i][k])
                else: qvalues[i][j][k] = -1*Parameters.inf

    # for i in range(N):
    #     print("source: ",(i+1))
    #     for j in range(N):
    #         s = ""
    #         for k in range(N):
    #             s  = s + " "+str(qvalues[i][j][k])
    #         print(s)
    return


def findAction(src,dest,edgeweights,sdist,qvalues):
    print(" find Action --> source ",src," dest ",dest)

    if src == dest:
        return dest


    N = traci.junction.getIDCount()
    mindist = Parameters.inf
    minneigbour = -1
    #print("src type: "+str(type(src))+" des type: "+str(type(dest)))

    for i in range(N):
        if edgeweights[src][i] == 1 and  sdist[i][dest] < mindist:
            mindist = sdist[i][dest]
            minneigbour = i

    print("mindistance,minneigbour ",mindist,minneigbour)

    list_greater = []
    list_less = []
    maxqvalue = -1*Parameters.inf
    for i in range(N):
        if edgeweights[src][i] == 0 or sdist[i][dest] >= Parameters.inf:
            continue
        if sdist[i][dest] > mindist*Parameters.delta:
            list_greater.append([i,qvalues[src][dest][i]])
        else:
            if qvalues[src][dest][i] > maxqvalue:
                maxqvalue = qvalues[src][dest][i]
                list_less.append([i,qvalues[src][dest][i]])

    print("list_less: \n",list_less);
    print("list_greater: \n",list_greater);

    maxqvalue = -1*Parameters.inf
    maxqvalneigbour = -1
    if list_less: #if list_less is not empty
        r = random.uniform(0,1)
        if r > Parameters.epsilon: #expolitation
            for x in list_less:
                if x[1] > maxqvalue:
                    maxqvalue = x[1]
                    maxqvalneigbour = x[0]
        else: #exploration
            #print("length of list_less: ",len(list_less))
            r = random.randint(0,len(list_less)-1) #note random.randint(a,b) ==> [a,b] and not [a,b)
            #print("random index: ",r)
            maxqvalneigbour = list_less[r][0]
    else:
        for x in list_greater:
            if x[1] > maxqvalue:
                maxqvalue = x[1]
                maxqvalneigbour = x[0]
    assert maxqvalneigbour != -1, "maxqvalneigbour not calculated properly" 
    return maxqvalneigbour

def congestedDistance(d,congestion):
    """ return d' = d*(1+a*(c)^b)"""
    return d*(1+Parameters.cona*pow(congestion,Parameters.conb))

def calCongestion(src,dest,nexthop,sdist,qvalues):
    """estimates expected congestion that the vehicle will
        encounter going from src to des via nexthop
    """
    #greedyly estimates the congestion of edge (src,nexthop) as espected congestion 
    edgeid = str(src)+"to"+str(nexthop)
    print("caluclated congestion: ",traci.edge.getLastStepOccupancy(edgeid))
    return traci.edge.getLastStepOccupancy(edgeid)



def calReward(src,dest,nexthop,sdist,qvalues):
    """
        Calculates reward for going to nexthop from src whn objective is to go to dest
    """
    #assert 1 == 2, "calculating reward function"
    adv = 0.0
    conadv = 0.0

    advsum = 0.0
    numneighbor = 0
    N = traci.junction.getIDCount()
    for i in range(N):
        if edgeweights[src][i] > 0 and sdist[i][dest] < Parameters.inf:
            advsum = advsum + (sdist[src][dest] - sdist[i][dest])
            numneighbor = numneighbor + 1
    advavg = advsum / numneighbor
    adv = (sdist[src][dest] - sdist[nexthop][dest])/advavg

    congestion = calCongestion(src,dest,nexthop,sdist,qvalues)
    conadvsum = 0.0
    numneighbor = 0
    for i in range(N):
        if edgeweights[src][i] > 0 and sdist[i][dest] < Parameters.inf:
            conadvsum = conadvsum + congestedDistance(sdist[src][dest],congestion) - congestedDistance(sdist[i][dest],congestion)
            numneighbor = numneighbor + 1
    conadvavg = conadvsum / numneighbor
    conadv = (congestedDistance(sdist[src][dest],congestion) - congestedDistance(sdist[nexthop][dest],congestion) )/conadvavg

    return Parameters.beta*adv + (1.0 - Parameters.beta)*conadv

def processVehicle(vid,route,curedge,edgeweights,sdist,qvalues):
    '''process the vid return next edge to be taken and reroute
        by updating the qvalues
    '''

    L = len(route)
    src = int(curedge.split("to")[1])
    dest = int(route[L-1].split("to")[1])
    src = src - 1
    dest = dest - 1

    if src == dest: #reached destination
        #assert route[0] == curedge, "single route element not equal to cur edge"
        '''
            store the travel time etc in a queue
        '''
        return None
 

    action = findAction(src,dest,edgeweights,sdist,qvalues)
    print("(src = %i,destination = %i ,action = %d): "%(src,dest,action))

    

    #update qvalues for all Q(s,a) pairs
    #Q(s,a) = Q(s,a) + alpha(r+ gamma(Max(Q(s',a')-Q(s,a))))

    naction = findAction(action,dest,edgeweights,sdist,qvalues)
    print("next action is: ",naction)
    N = traci.junction.getIDCount()
    for i in range(N):
        if edgeweights[src][i] > 0 and sdist[i][dest] < Parameters.inf:
            # assert 1 == 2,"ki hochche"
            # print("jai ma")
            qvalues[src][dest][i] = qvalues[src][dest][i] + Parameters.alpha*(calReward(src,dest,action,sdist,qvalues)
                                    + Parameters.gamma*qvalues[action][dest][naction] - qvalues[src][dest][i])

    return action+1


def run(edgeweights,sdist,qvalues):
    """execute the TraCI control loop"""
    step = 0
    edges = traci.edge.getIDList();
    print("num edgees = %i",traci.edge.getIDCount())
    print("edge list",edges)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep() 
        print("running step no. %i",step)

        numedges = 24
        for i in range(numedges):
            loopid = "loop"+str(i)
            vids = traci.inductionloop.getLastStepVehicleIDs(loopid)
            src = (edges[i].split('to')[1])
            curedge = edges[i]
            if vids :
                print(''.join(vids))
                for vid in vids:
                    route = traci.vehicle.getRoute(vid)
                    print(vid+" old route: "+' '.join(route))
                    dest = int(route[len(route)-1].split('to')[1])
                    nexthop = processVehicle(vid,route,edges[i],edgeweights,sdist,qvalues)

                    if nexthop  and nexthop != dest: #currently in last edge so do nothing
                        print("nexthop: ",nexthop," dest: ",dest,nexthop == dest)
                        nextedge = str(src)+"to"+str(nexthop)
                        rid = "r"+str(nexthop)+"to"+str(dest)
                        print("new route id: ",rid)
                        routeedges = traci.route.getEdges(rid)
                        routeedges.insert(0,nextedge)
                        routeedges.insert(0,curedge)
                        print("new route edges: ",routeedges)
                        traci.vehicle.setRoute(vid,routeedges)
                        newroute = traci.vehicle.getRoute(vid)
                        print(vid+" new route: "+' '.join(newroute))
        step += 1

    traci.close()
    sys.stdout.flush()
    return


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "testdata/sim.sumocfg",
                             "--tripinfo-output", "simtripinfo.xml"])


    N = traci.junction.getIDCount()
    edgeweights = [[0,1,1,1,0,0,0,0,0,0],
            [0,0,1,1,1,0,0,0,0,0],
            [0,0,0,1,1,1,0,0,0,0],
            [0,0,0,0,1,1,1,0,0,0],
            [0,0,0,0,0,1,1,1,0,0],
            [0,0,0,0,0,0,1,1,1,0],
            [0,0,0,0,0,0,0,1,1,1],
            [0,0,0,0,0,0,0,0,1,1],
            [0,0,0,0,0,0,0,0,0,1],
            [0,0,0,0,0,0,0,0,0,0]]
    sdist = [[Parameters.inf for j in range(N)] for i in range(N)]
    qvalues = [[[0 for k in range(N)] for j in range(N)] for i in range(N)]

    preProcess(edgeweights,sdist,qvalues)

    #print("sdist: \n",DataFrame(sdist))
    #print("qvalues:\n",DataFrame(qvalues))

    run(edgeweights,sdist,qvalues)
