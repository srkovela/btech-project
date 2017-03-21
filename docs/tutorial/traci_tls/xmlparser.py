#!/bin/python

from xml.dom import minidom
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt

# xmldoc = minidom.parse('simtripinfo.xml')
# itemlist = xmldoc.getElementsByTagName('tripinfo')
# departlist = []
# durationlist = []
# idlist = []
# for s in itemlist:
# 	departlist.append(float(s.attributes['depart'].value))
# 	durationlist.append(float(s.attributes['duration'].value))
# 	t = s.attributes['id'].value
# 	t = t.split('#')
# 	src = int(t[2])
# 	des = int(t[3])
# 	route_id = (src-1)*3+(des-8)
# 	idlist.append(route_id)
 
# #print  len(durationlist)," ",len(departlist)
# df = pd.DataFrame({'depart_time':departlist,'duration':durationlist,'id':idlist})
# #print df.head()

# for i in range(9):
# 	print df[df['id'] == i]
# 	print df[df['id'] == i].plot(x='depart_time',y='duration',style='o')
# 	#df[df['id'] == i].mean()
# 	plt.show()

# print len(df.groupby('depart_time').mean())
# df.plot(x='depart_time',y='duration',style='o')

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
 
print  len(durationlist)," ",len(departlist)
df = pd.DataFrame({'depart_time':departlist,'duration':durationlist,'id':idlist})
print "Average Travel Duration: ",df['duration'].mean()


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
 
print  len(durationlist)," ",len(departlist)
df = pd.DataFrame({'depart_time':departlist,'duration':durationlist,'id':idlist})
print "Average Travel Duration: ",df['duration'].mean()

