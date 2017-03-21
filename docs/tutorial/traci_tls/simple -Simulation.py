
#To find average travel time by 
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


# In[37]:

class Parameters:
    inf = 1e8
    delta = 1.5 #mindist multiplier
    beta = 0.5 #min travel tims vs min path trade off
    alpha = 0.1 #learning rate
    gamma = 0.5 #discount factor
    
    epsilon =  0.5 #exploartion vs exploitation
    cona = 10 #constant multiplier for congestion
    conb = 4 #length increases quadratically to the congestion
    iterations = 1000


# In[10]:

#Generates Route file for example graph



def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 100  # number of initial vehicles
    # demand per second from different directions
    vehNr = 0
    with open("testdata/sim.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="vtype0" accel="0.8" decel="4.5" sigma="0.5" length="2" minGap="1" maxSpeed="16.67" guiShape="passenger"/>
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
        <route id="r3to8" edges="3to5 5to8 " />
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

        print("Vehicles Generated: ",pd.DataFrame(count))
        print("</routes>", file=routes)


# # In[43]:

# def preProcess(g,w):
#     N = len(g)
#     dist = [[Parameters.inf for j in range(N)]for i in range(N)]
#     for i in range(N): #for all vertices
#         dist[i][i] = 0
#         pq = PriorityQueue()
#         pq.put((0,i))
#         while pq.qsize() > 0:
#             u = pq.get()[1]
#             #print "u: ",u
#             for j in range(len(g[u])):
#                 v = g[u][j]
#                 d = w[u][j]
#                 if dist[i][v] > dist[i][u] + d:
#                     dist[i][v] = dist[i][u] + d
#                     pq.put((dist[i][v],v))
    
#     #Find Initial Qvalues
#     qf = [[[dist[i][j] for k in range(N)]for j in range(N)] for i in range(N)]
#     prev = [[[0 for k in range(N)]for j in range(N)] for i in range(N)]

# #     alpha = 0.01
# #     gamma = 0.5
#     alpha = Parameters.alpha
#     gamma = Parameters.gamma
#     for iteration in range(1000):
#         for src in range(N):
#             for des in range(N):
#                 if src == des:
#                     continue
#                 else:
#                     for action in range(len(g[src])):
#                         s_dash = g[src][action]
#                         reward = -(w[src][action] + dist[g[src][action]][des])
#                         maxim = -1e100
#                         if s_dash == des:
#                             maxim = 100 - qf[src][des][s_dash]
#                         else:
#                             for a_dash in range(len(g[s_dash])):
#                                 maxim = max(maxim,qf[s_dash][des][g[s_dash][a_dash]] -  qf[src][des][g[src][action]])
#                         qf[src][des][g[src][action]] =  qf[src][des][g[src][action]] + Parameters.alpha*(reward + Parameters.gamma*maxim)


#         maxdiff = -1e8 
#         for i in range(N):
#             for j in range(N):
#                 for k in range(len(g[i])):
#                     diff = abs(prev[i][j][g[i][k]]-qf[i][j][g[i][k]])
#                     maxdiff = max(maxdiff,diff)
#                     prev[i][j][g[i][k]] = qf[i][j][g[i][k]]
#         if maxdiff < .0001:
#             print ("coverged with in expected deviation in #iterations: ",iteration+1)
#             break
#     return dist,qf


# # In[ ]:

# def findAction(src,dest,w,dist,qf,g):
#     action = -1
#     maxim = -1*Parameters.inf
#     for i in range(len(g[src])):
#         if dist[g[src][i]][dest] != Parameters.inf and qf[src][dest][g[src][i]] > maxim:
#             maxim = qf[src][dest][g[src][i]]
#             action = g[src][i]
#     return action


# In[ ]:
#Hard code 
# def conDistance(x,c):
#     return x*((1+3*c)**4)

# #src = [0-3] des = [7-9]
# def calCongestion(src,dest,nexthop,g,w):
#     edgeid = str(src+1)+"to"+str(nexthop+1)
#     congestion = traci.edge.getLastStepOccupancy(edgeid)
#     print("Edgeid: ",edgeid," Congestion: ",congestion)
#     return congestion


# def processVehicle(vid,route,curedge,w,dist,qf,g):
#     '''process the vid return next edge to be taken and reroute
#         by updating the qvalues
#     '''

#     L = len(route)
#     src = int(curedge.split("to")[1])
#     dest = int(route[L-1].split("to")[1])
#     src = src - 1
#     dest = dest - 1

#     if src == dest: #reached destination
#         #assert route[0] == curedge, "single route element not equal to cur edge"
#         '''
#             store the travel time etc in a queue
#         '''
#         return None
#     else:
#         print("Src: ",src,"dist: ",dest)
#         print("Print Q-Values Before update:=============== \n",qf[src][dest],"\n===================")

#         # assert 1 == 2, "Processing Vehicles"
#         #update qvalues for all Q(s,a) pairs
#         #Q(s,a) = Q(s,a) + alpha(r+ gamma(Max(Q(s',a')-Q(s,a))))
#         for action in range(len(g[src])):
#             s_dash = g[src][action]

#             #Calculate Rewrad Function Based On 
#             congestion = calCongestion(src,dest,s_dash,g,w)
#             print("Congestion New: ",congestion,"dist: ",(w[src][action]+dist[g[src][action]][dest])," con dist: ",conDistance(w[src][action]+dist[g[src][action]][dest],congestion))

#             reward = (-1*Parameters.beta*(w[src][action] + dist[g[src][action]][dest])
#             -1*(1 - Parameters.beta)*(conDistance(w[src][action]+dist[g[src][action]][dest],congestion)) )
#             maxim = -1e100
#             if s_dash == dest:
#                 maxim = 100 - qf[src][dest][s_dash]
#             else:
#                 for a_dash in range(len(g[s_dash])):
#                     maxim = max(maxim,qf[s_dash][dest][g[s_dash][a_dash]] -  qf[src][dest][g[src][action]])
#             qf[src][dest][g[src][action]] =  qf[src][dest][g[src][action]] + Parameters.alpha*(reward + Parameters.gamma*maxim)

#         naction = findAction(src,dest,w,dist,qf,g)
#         print("(src = %i,destination = %i ,action = %d): "%(src,dest,naction))

#         print("Print Q-Values After update:=============== \n",qf[src][dest],"\n===================")
#         return naction+1


# # In[ ]:

#Route Choice Server
def run(w,dist,qf,g):
    """execute the TraCI control loop"""
    print("Control loop")
    step = 0
    count = [[0 for j in range(3)] for i in range(3)]

    vehNr = 100
    edges = traci.edge.getIDList();
    print("num edgees = %i",traci.edge.getIDCount())
    print("edge list",edges)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep() 
        print("running step no. %i",step)

        # numedges = 24
        # for i in range(numedges):
        #     loopid = "loop"+str(i)
        #     vids = traci.inductionloop.getLastStepVehicleIDs(loopid)
        #     src = (edges[i].split('to')[1])
        #     curedge = edges[i]
        #     if vids :
        #         print(''.join(vids))
        #         for vid in vids:
        #             route = traci.vehicle.getRoute(vid)
        #             print(vid+" old route: "+' '.join(route))
        #             dest = int(route[len(route)-1].split('to')[1])
                    
        #             #Get New Route Suggestion
        #             nexthop = processVehicle(vid,route,edges[i],w,dist,qf,g)

        #             if nexthop  and nexthop != dest: #currently in last edge so do nothing
        #                 print("nexthop: ",nexthop," dest: ",dest,nexthop == dest)
        #                 nextedge = str(src)+"to"+str(nexthop)
        #                 rid = "r"+str(nexthop)+"to"+str(dest)
        #                 print("new route id: ",rid)
        #                 routeedges = traci.route.getEdges(rid)
        #                 routeedges.insert(0,nextedge)
        #                 routeedges.insert(0,curedge)
        #                 print("new route edges: ",routeedges)
        #                 traci.vehicle.setRoute(vid,routeedges)
        #                 newroute = traci.vehicle.getRoute(vid)
        #                 print(vid+" new route: "+' '.join(newroute))
        if step <= 500:            
            #Add New Vehicles according to need
            print("Dynamic Vehicles: ==================================")
            for x in range(10):
                src = random.randint(1,3)
                dest = random.randint(8,10)
                vid = "vid#"+str(vehNr)+'#'+str(src)+'#'+str(dest)
                rid = "r"+str(src)+"to"+str(dest)
                traci.vehicle.add(vid,rid,typeID="vtype1",depart = step,lane = 0)
                vehNr += 1
        #         route = traci.vehicle.getRoute(vid)
        #         nexthop = processVehicle(vid,route,"xto"+str(src),w,dist,qf,g)

        #         print("src: %i des: %i NextHop: %i"%(src,dest,nexthop))

        #         if nexthop:
        #             nextedge = str(src)+"to"+str(nexthop)
        #             rid = "r"+str(nexthop)+"to"+str(dest)
        #             print("new route id: ",rid)
        #             routeedges = traci.route.getEdges(rid)
        #             print("Route edges of new route: ",routeedges)
        #             routeedges.insert(0,nextedge)
        #             #routeedges.insert(0,curedge)
        #             print("new route edges: ",routeedges)
        #             traci.vehicle.setRoute(vid,routeedges)
        #             newroute = traci.vehicle.getRoute(vid)
        #             print(vid+" new route: "+' '.join(newroute))
        #         else:
        #             assert 1 == 2
        #         print("Dynamic Vehicle End ==================================")

        #         vehNr += 1
        #         count[src-1][dest-8] = count[src-1][dest-8]+1; #input specific


        #         for x in range(10):
        #             src = random.randint(3,3)
        #             dest = random.randint(9,9)
        #             vid = "vid#"+str(vehNr)+'#'+str(src)+'#'+str(dest)
        #             rid = "r"+str(src)+"to"+str(dest)
        #             traci.vehicle.add(vid,rid,typeID="vtype1",depart = step,lane = 0)
        #             route = traci.vehicle.getRoute(vid)
        #             nexthop = processVehicle(vid,route,"xto"+str(src),w,dist,qf,g)

        #             print("src: %i des: %i NextHop: %i"%(src,dest,nexthop))

        #             if nexthop:
        #                 nextedge = str(src)+"to"+str(nexthop)
        #                 rid = "r"+str(nexthop)+"to"+str(dest)
        #                 print("new route id: ",rid)
        #                 routeedges = traci.route.getEdges(rid)
        #                 print("Route edges of new route: ",routeedges)
        #                 routeedges.insert(0,nextedge)
        #                 #routeedges.insert(0,curedge)
        #                 print("new route edges: ",routeedges)
        #                 traci.vehicle.setRoute(vid,routeedges)
        #                 newroute = traci.vehicle.getRoute(vid)
        #                 print(vid+" new route: "+' '.join(newroute))
        #             else:
        #                 assert 1 == 2
        #             print("Dynamic Vehicle End ==================================")

        #             vehNr += 1
        #             count[src-1][dest-8] = count[src-1][dest-8]+1; #input specific

        step += 1
    return


# In[47]:
def processData():
    xmldoc = minidom.parse('simple-simtripinfo.xml')
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


    # for i in range(9):
    #     print("route "+str(i/3+1)+" to "+str(i%3+8)," : ",df[df['id'] == i]['duration'].mean())
    #     df[df['id'] == i].plot(x='depart_time',y='duration',style='o',title="route "+str(i/3+1)+" to "+str(i%3+8))
    #     plt.xlabel('departure time')
    #     plt.ylabel('average travel duration')
    #     plt.show()


if __name__ == "__main__":
        sumoBinary = checkBinary('sumo-gui')
        file = open("log.txt",'w')
        sys.stdout = file

        #genrate route file
        generate_routefile()
        
        #Start Simulation
        traci.start([sumoBinary, "-c", "testdata/sim.sumocfg",
                             "--tripinfo-output", "simple-simtripinfo.xml"])
        
        
        g=[[1,2,3],[2,3,4],[3,4,5],[4,5,6],[5,6,7],[6,7,8],[7,8,9],[8,9],[9],[]]
        #change weight test
        w = [[10, 10, 1], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10], [10, 10, 10], [1, 10], [1], []]      
        N = len(g)
        #dist,qf = preProcess(g,w)
#         display(pd.DataFrame(dist))
#         display(pd.DataFrame(qf))
        

        # print("Print Q-Values Initial:=============== \n",qf[0][8],"\n===================")
        # print("Print Q-Values Initial:=============== \n",qf[2][8],"\n===================")
        qf = []
        dist = []
        run(w,dist,qf,g)
        print("simulation completed successfully!!")
        
        traci.close()
        sys.stdout.flush()
        print("simulation completed successfully!!")

        #display(qf)
        #processData()


