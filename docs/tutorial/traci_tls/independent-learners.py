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
from xml.dom import minidom
import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
import numpy as np
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
    alpha = 0.5 #learning rate
    epsilon =  0.5 #exploartion vs exploitation
    gamma = 0.4 #discount factor
    cona = 10 #constant multiplier for congestion
    conb = 4 #length increases quadratically to the congestion
    max_iterations = 1000
    congestion_threshold = 0.2
    epoch_time = 30


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
     
    #print  len(durationlist)," ",len(departlist)
    df = pd.DataFrame({'depart_time':departlist,'duration':durationlist,'id':idlist})
    print(df.head())

    for i in range(9):
        print("route "+str(i/3+1)+" to "+str(i%3+8)," : ",df[df['id'] == i]['duration'].mean())
        df[df['id'] == i].plot(x='depart_time',y='duration',style='o',title="route "+str(i/3+1)+" to "+str(i%3+8))
        plt.xlabel('departure time')
        plt.ylabel('average travel duration')
        plt.show()

def generate_routefile(count):
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
        <route id="r1to10" edges="1to4 4to5 5to6 6to7 7to8 8to9 9to10 " />
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
        vehNr = 0
        for i in range(len(count)):
            for j in range(len(count[0])):
                for k in range(count[i][j]):
                    src = i+1
                    dest = j+8
                    rid = "r"+str(src)+"to"+str(dest)
                    print('    <vehicle id="vid#%i#%i#%i" type="vtype1" route="%s" depart="%i" />' % (
                    vehNr,src,dest,rid,0), file=routes)
                    vehNr += 1

        print("Vehicles Generated: ",pd.DataFrame(count))
        print("</routes>", file=routes)
#=====================================================================================
def printQValues(qvalues):
    print("printing Qvalues=====================")
    N = traci.junction.getIDCount()
    for i in range(N):
        print("source: ",(i+1))
        for j in range(N):
            s = ""
            for k in range(N):
                s  = s + " "+str(qvalues[i][j][k])
            print(s)

def findOptimalStrategy(qf,g,dist,w):
    alpha = Parameters.alpha
    gamma = Parameters.gamma
    # qf = [[[0 for k in range(N)] for j in range(N)] for i in range(N)]
    prev = [[[0 for k in range(N)] for j in range(N)] for i in range(N)]
    for iteration in range(Parameters.max_iterations):
        for i in range(N):
            for j in range(N):
                if i == j: #src == dest
                    continue
                else:
                    for k in range(len(g[i])): #Q(S_t,a_t)
                        v = g[i][k] #S_t+1
                        d = w[i][k] + dist[v][j] #reward naive
                        if v == j:
                            maxq = 100 - qf[i][j][v] #Crucial
                        else:
                            maxq = -1e8
                            for l in range(len(g[v])):
                                maxq = max(maxq,qf[v][j][g[v][l]] - qf[i][j][v]) #Max_a'(Q(s_t+1,a')-Q(s_t,a))
                        qf[i][j][v] = qf[i][j][v] + alpha*(-1*d + gamma*maxq)
            #Check Covergence
            maxdiff = -1e8 
            for i in range(N):
                for j in range(N):
                    for k in range(N):
                        diff = abs(prev[i][j][k]-qf[i][j][k])
                        maxdiff = max(maxdiff,diff)
                        prev[i][j][k] = qf[i][j][k]
        if maxdiff < .00001:
            print("coverged with in expected deviation in #iterations: ",iteration+1)
            break
                        
    pd.DataFrame(qf)

#=====================================================================================================

class Vehicle:
    def __init__(self):
        self.edge_time_map = dict()

    def put(self,edge,time):
        self.edge_time_map[edge] = time

    def get_edge_time_map(self):
        return self.edge_time_map

    def get_travel_time(self):
        total_time = 0
        for key in edge_time_map.keys():
            total_time += self.edge_time_map[key]
        return total_time


def preProcess(edgeweights,sdist,qvalues):
    N = traci.junction.getIDCount() #no. of junctions
    print("no. of nodes: ",N)
    '''write djikstra'''
    pq = PriorityQueue()
    for i in range(N):
        pq.put((0,i))
        sdist[i][i] = 0
        while pq.qsize() > 0:
            top = pq.get()
            #print("top: ",top)
            u = top[1]
            for j in range(N):
                if  edgeweights[u][j] > 0 and sdist[i][u]+edgeweights[u][j] < sdist[i][j]:
                    sdist[i][j] = sdist[i][u] + edgeweights[u][j]
                    pq.put((sdist[i][j],j))

        sdist[i][i] = Parameters.inf
    print(pd.DataFrame(sdist))
    return


def ta(expected):
    fa = 1.0
    alpha = 1.0
    ca = 10
    beta = 2.0
    return fa*(1+alpha*((float(expected)/ca)**beta))


def update_qtable(qvalues,mp,vid,src,des,edgeToId,etp,edgeweights):
    print("updating Q table")
    vehicle = mp[vid]
    edge_time_map = vehicle.get_edge_time_map()
    des = des - 7
    src = src - 0
    print("In update Q table: ",src,des)
    total_time_taken = 0
    for key in edge_time_map.keys():
        print("Edge: ",key[0]+1," -->",key[1]+1," time: ",edge_time_map[key])
        total_time_taken += edge_time_map[key]
    print("total_time: ",total_time_taken)

    alpha = 0.5
    gamma = 0.4
    
    print("Edges present in vid: ",vid)
    for key in edge_time_map.keys():
        print("Edge: ",key)

        edgeid = edgeToId[key]
        congestion_cost = 0
        time_taken = float(edge_time_map[key]*total_time_taken)/etp[src][des]
        reward = -1*time_taken + congestion_cost
        print("Reward: ",reward)

        if qvalues[src][des][edgeid] == -1*Parameters.inf:
            print("qvalue Made zero")
            qvalues[src][des][edgeid] = 0

        temp = qvalues[src][des][edgeid]
        #change this to find max only among the possible edges from previous edge
        
        #chooses maxq value from the availabale edges from this edge
        a_src = key[0]
        a_des = key[1]
        maxaction = -1*Parameters.inf
        for i in range(len(edgeweights[a_des])):
            if edgeweights[a_des][i] > 0:
                t_edge = (a_des,i)
                t_edge_id = edgeToId[t_edge]
                maxaction = max(maxaction,qvalues[src][des][t_edge_id])

        
        #Chooses max qvalue fro all edges
        #maxaction = np.max(np.array(qvalues[src][des]))       
        if maxaction <= -1*Parameters.inf:
            print("Max action made zero")
            maxaction = 0

        qvalues[src][des][edgeid] = qvalues[src][des][edgeid] + alpha*(reward + gamma*maxaction)
        print("old: ",temp," new: ",qvalues[src][des][edgeid])
    return qvalues



def cal_threshold(step,upper,lower,iterations):
    ratio = float(lower)/upper
    return upper*(ratio**(float(step)/iterations))


def run(edgeweights,sdist,qvalues,etp):
    """execute the TraCI control loop"""
    mp = dict()
    edgeToId = dict()
    IdToEdge = dict()
    edges = ['1to2', '1to3', '1to4', '2to3', '2to4', '2to5', '3to4', '3to5', '3to6', '4to5', '4to6', '4to7', '5to6', '5to7', '5to8', '6to7', '6to8', '6to9', '7to8', '7to9', '7to10', '8to9','8to10', '9to10']
    print("num edgees = %i",traci.edge.getIDCount())
    print("edge list",edges)
    numedges = 24
    duplicate = dict()

    upper = 0.2
    lower = 0.05
    iterations = 1000

    for i in range(numedges):
        src = int(edges[i].split('to')[0]) -1
        des = int(edges[i].split('to')[1]) -1
        edgeToId[(src,des)] = i
        IdToEdge[i] = (src,des)

    uc = 0
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep() 
        numVehiclesMap = dict()
        for i in range(numedges):
            src = int(edges[i].split('to')[0]) -1
            des = int(edges[i].split('to')[1]) -1
            numVehiclesMap[(src,des)] =  traci.edge. getLastStepVehicleNumber(edges[i])

        step += 1
        
        threashold = max(lower,cal_threshold(step,upper,lower,iterations))


        for i in range(numedges):
            loopid = "loop"+str(i)
            #vids = traci.edge.getLastStepVehicleIDs(edges[i])
            vids = traci.inductionloop.getLastStepVehicleIDs(loopid)
            numVehicles = traci.edge.getLastStepVehicleNumber(edges[i])

            if vids :
                print("No. of vehicles in edge: ",edges[i]," ",numVehicles)
                print("Loop vids: ",' '.join(vids))
                
                presrc = int(edges[i].split('to')[0])-1
                src = int(edges[i].split('to')[1])-1
                curedge = edges[i]

                for vid in vids:
                    if duplicate.get(vid) != None and duplicate[vid] == curedge:
                        continue
                    duplicate[vid] = curedge

                    print("Current vid processed: ",vid)

                    route = traci.vehicle.getRoute(vid)

                    origin = int(vid.split('#')[2])-1
                    destination = int(vid.split('#')[3])-1
                    des = int(route[len(route)-1].split('to')[1])-1
                    predes = int(route[len(route)-1].split('to')[0])-1
                    print(" old route: "+' '.join(route)+" src: ",src+1," des: ",des+1," origin: ",origin+1," curedge: ",curedge)

                    if src == des:
                        pass
                    else:
                        print("Rerouting based on Q table")

                        #update the last edge travel time
                        try:
                            vehicle = mp[vid]
                        except:
                            mp[vid] = Vehicle()
                            vehicle = mp[vid]

                        print("Numvehicles: ",numVehiclesMap[(presrc,src)],"Ta: ",ta(numVehiclesMap[(presrc,src)]))
                        vehicle.put((presrc,src),ta(numVehiclesMap[(presrc,src)]))

                        possible_actions = []
                        possible_qvalues = []
                        for action in range(len(edgeweights[src])):
                            if edgeweights[src][action] > 0:
                                possible_actions.append(action)
                                possible_qvalues.append((qvalues[origin-0][destination-7][edgeToId[(src,action)]],action))
                        epsilon = random.uniform(0,1)

                        print("Possible qvalues: ",possible_qvalues)
                        print("possible action list: ",possible_actions)
                        print("epsilon value: ",epsilon)

                        if epsilon < threashold:
                            print("random action")
                            nexthop = possible_actions[random.randint(0,len(possible_actions)-1)]
                        else:
                            print("maxq value")
                            print("Sorted qvalue list: ",sorted(possible_qvalues,key = lambda x:x[0]))
                            nexthop = list(sorted(possible_qvalues,key = lambda x:x[0]))[len(possible_qvalues)-1][1]

                        print("Action chosen: ",nexthop+1," at node: ",src+1)#" to go to",des+1)
                        
                        print("nexthop: ",nexthop+1," dest: ",des+1)
                        if nexthop == des:
                            vehicle.put((src,des),ta(numVehiclesMap[(src,des)]))
                            qvalues = update_qtable(qvalues,mp,vid,origin,des,edgeToId,etp,edgeweights)
                            print("Updated qvalues: ",qvalues[origin-0][destination-7])
                            uc = uc+1
                        else:
                            rid = "r"+str(nexthop+1)+"to"+str(des+1)
                            print("new route id: ",rid)
                            nextedge = str(src+1)+"to"+str(nexthop+1)
                            routeedges = traci.route.getEdges(rid)
                            routeedges.insert(0,nextedge)
                            routeedges.insert(0,curedge)
                            print("new route edges: ",routeedges)
                            traci.vehicle.setRoute(vid,routeedges)
                            newroute = traci.vehicle.getRoute(vid)
                            print(vid+" new route: "+' '.join(newroute))

                        print("---------------------------------------------------------")
                    print("===============================================================")
                print("step no: ",step," threashold: ",threashold)

    print("converged actions: ============== ")
    for i in range(10):
        maxaction = -1
        maxval = -1*Parameters.inf
        all_vals = []
        for j in range(len(edgeweights[i])):
            if edgeweights[i][j] > 0:
                edge = (i,j)
                edgeid = edgeToId[edge]
                all_vals.append((j+1,qvalues[0][2][edgeid]))
                if qvalues[0][2][edgeid] > maxval:
                    maxaction = j
                    maxval = qvalues[0][2][edgeid]
        print("All the actions,values: ",all_vals)
        print("Best action at node: ",i+1," is: ",maxaction+1,"with val: ",maxval)

    print("update count: ",uc)
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

    print("filename: ",__file__)
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')


    random.seed(50)  # make tests reproducible
    file = open("log.txt",'w')
    sys.stdout = file

    #expected no. of vehicles in od pairs
    exveh = [[0 for j in range(3)] for i in range(3)]
    etp = [[0 for j in range(3)] for i in range(3)]
    etp[0][2] = 3
    exveh[0][2] = 1000
    # first, generate the route file for this simulation
    generate_routefile(exveh)
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "testdata/sim.sumocfg",
                             "--tripinfo-output", "simtripinfo.xml"])


    N = traci.junction.getIDCount()
    numedges = 24

    g=[[1,2,3],[2,3,4],[3,4,5],[4,5,6],[5,6,7],[6,7,8],[7,8,9],[8,9],[9],[]]
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
    qvalues = [[[-1*Parameters.inf for k in range(numedges)] for j in range(3)] for i in range(3)]
    w = [[] for i in range(len(g))]
    for i in range(len(g)):
        for j in range(len(g[i])):
            w[i].append(10)

    preProcess(edgeweights,sdist,qvalues)
 
    run(edgeweights,sdist,qvalues,etp)
    traci.close()
    sys.stdout.flush()

    print("simulation completed successfully!!")

    #processData()
