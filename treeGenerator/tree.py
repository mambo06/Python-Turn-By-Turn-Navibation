
# coding: utf-8

# In[2]:


import geopandas as gpd
from shapely.geometry import Point
from geopy.distance import lonlat, distance
import networkx as nx
import pickle
from shapely.ops import unary_union
import scipy

states = gpd.read_file('../dataset/Pedestrian_network.json')


#geoms = states.geometry.tolist()
G = nx.Graph()
nodes_list = {}
n = 0
for index, line in states.iterrows():
    #print(list(line['geometry'].coords))
    for seg_start, seg_end in zip(list(line['geometry'].coords),list(line['geometry'].coords)[1:]):
        G.add_edge(seg_start, seg_end,
                   weight = line['Shape_Length'] * line['COST'],
                  idx = line['OBJECTID'])



geoms = unary_union(states.geometry.tolist())


nodes = []
for line in geoms:
   for seg_start, seg_end in zip(list(line.coords),list(line.coords)[1:]):
    nodes.append((seg_start[1],seg_start[0]))
    nodes.append((seg_end[1],seg_end[0]))
    #G.add_edge(seg_start, seg_end)
nodes = set(nodes)
nodes = [list(each) for each in nodes]


# dump nodes for back up
pickle_out = open("../pickle/N.pickle","wb")
pickle.dump(nodes, pickle_out)
pickle_out.close()

# dump networkX graph for routing
pickle_out = open("../pickle/G.pickle","wb")
pickle.dump(G, pickle_out)
pickle_out.close()




dTree = scipy.spatial.KDTree(nodes)

# dump dTree for geocode, future
pickle_out = open("../pickle/D.pickle","wb")
pickle.dump(dTree, pickle_out)
pickle_out.close()

