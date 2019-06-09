#!/usr/bin/env python3

from flask import Flask, request, jsonify
from flask_restful import Resource, Api
import json
import numpy as np
import geopandas as gpd
import networkx as nx
import pickle
import scipy

#formats longlat
def angles(A,B,C):
    A = (A[1],A[0])
    B = (B[1],B[0])
    C = (C[1],C[0])
    a = np.radians(np.array(A))
    b = np.radians(np.array(B))
    c = np.radians(np.array(C))
    avec = a - b
    cvec = c - b
    lat = b[0]
    avec[1] *= math.cos(lat)
    cvec[1] *= math.cos(lat)

    return np.degrees(
        math.acos(np.dot(avec, cvec) / (np.linalg.norm(avec) * np.linalg.norm(cvec))))

def calcDir(p0, p1, p2):
  v1y = float(p1[0]) - float(p0[0])
  v1x = float(p1[1]) - float(p0[1])
  v2y = float(p2[0]) - float(p1[0])
  v2x = float(p2[1]) - float(p1[1])
  deg = v1x * v2y - v1y * v2x
  if  deg > 0.0:
      return 'right', deg
  else:
      return 'left', deg

from shapely.geometry import Point

def nearPoint(longLat, geoFrame, last = False):
    mloc = Point(longLat[1], longLat[0])
    if last == True :
        return list(geoFrame[geoFrame.
                index.isin(geoFrame.
                           geometry.distance(mloc).nsmallest(1).index.tolist())].geometry.iloc[0].coords)[-1]
    else:
        return list(geoFrame[geoFrame.
                index.isin(geoFrame.
                           geometry.distance(mloc).nsmallest(1).index.tolist())].geometry.iloc[0].coords)[0]

def calculate_initial_compass_bearing(pointA, pointB):
    pointA = (pointA[1],pointA[0])
    pointB = (pointB[1],pointB[0])

    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)
    
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return round(compass_bearing)




def swift(alist):
    temp = []
    for each in alist :
        temp.append((each[1],each[0]))
    return temp




import math
import polyline
def transformerSDK(aroute):
    if len(aroute) == 0 :
        return True
    #prepare summary data
    data = {}
    data = {
        "routeIndex" : 0,
        "distance" : 0, # tobe updated after each
        "duration" : 0, # tobe updated after each
        "geometry" : 0, # tobe updated after each
        "weight" : 0, # tobe updated after each
        "weight_name" : "routability",
        "legs" : [],
        "routeOptions" : {},
        "voiceLocale" : "en-US"
        
    }
    data["routeOptions"] = {
        "baseUrl":"http://http://127.0.0.1/sdk/",
        "user":"mapbox",
        "profile":"driving-traffic",
        "coordinates":[list(aroute[0]),list(aroute[-1])],
        "language":"en",
        "bearings":";",
        "continue_straight":True,
        "roundabout_exits":True,
        "geometries":"polyline6",
        "overview":"full",
        "steps":True,
        "annotations":"congestion,distance",
        "voice_instructions":True,
        "banner_instructions":True,
        "voice_units":"imperial",
        "access_token": "anystring",
        "uuid":"anystring"
    }
    data["legs"] = [{
        "distance" : 0,
        "duration" : 0,
        "summary" : "start,end",
        "steps" :[],
        "annotations":{}
    }]
    
    #add depart data
    data["legs"] = [{}]
    data["legs"] = [{"summary":"Start St, End St",
                     "weight":0, # to be updated in the end of each
                     "duration":0, # to be updated in the end of each
                     "steps":[]}]
    data["legs"][0]["steps"] = [{"distance":0, # tobe updated when a turn occur
                                 "intersections":[{"location":[aroute[0][0], aroute[0][1]],"bearings":[calculate_initial_compass_bearing(aroute[0], aroute[1])], "entry":[True], "out": 0, "in":0}], # leave blanks for now
                                 "driving_side":"left", # always left
                                 "geometry":0,
                                 "mode":"walking",
                                 "maneuver":{},
                                 "ref":0, 
                                 "weight":0, # tobe updated when a turn occur
                                 "duration":0, # tobe updated when a turn occur
                                 "name":"street name",
                                 "distance":0}] # tobe updated when a turn occur
    data["legs"][0]["steps"][0]["maneuver"] = {"bearing_after":calculate_initial_compass_bearing(aroute[0], aroute[1]), # tobe updated when a turn occur
                                               "bearing_before":0, # tobe updated when a turn occur
                                               "location":[aroute[0][0], aroute[0][1]],
                                               "type":"depart",
                                               "instruction":"Head on"}
    
    data["legs"][0]["steps"][0]["voiceInstructions"] = [{"distanceAlongGeometry":0,# tobe updated when a turn occur
                                                          "announcement":"Head on",
                                                          "ssmlAnnouncement":"\u003cspeak\u003e\u003camazon:effect name\u003d\"drc\"\u003e\u003cprosody rate\u003d\"1.08\"\u003eHead on\u003c/prosody\u003e\u003c/amazon:effect\u003e\u003c/speak\u003e"}]
    data["legs"][0]["steps"][0]["bannerInstructions"] = [{
        "distanceAlongGeometry":0, # tobe updated when a turn occur
        "primary":{
            "text":"Street",
            "components":[{
                "text":"Street",
                "type":"text",
                "abbr":True,
                "abbr_priority":0}
            ],
            "type":"turn", # tobe updated when a turn occur
            "modifier":"left"}}] # tobe updated when a turn occur
    data["legs"][0]["annotation"] = {"distance":[], "congestion":[]}
    
    # each per point data
    n = 0 # waypints index
    z = 0 # keep before point
    total_weight = 0
    total_duration = 0
    total_distance = 0
    prev_weight = 0
    prev_duration = 0
    prev_distance = 0
    n_turn = 0
    leg_points = []
    waypoints = []
    turn_points = [aroute[0]]
    
    for each in aroute:
        leg_points.append(each)
        waypoints.append(each)
        if z != 0 :
            # law of cosines, distance in km
            distance = math.acos(math.sin(math.radians(each[0]))*math.sin(math.radians(z[0]))+math.cos(math.radians(each[0]))*math.cos(math.radians(z[0]))*math.cos(math.radians(z[1])-math.radians(each[1])))*6371
            distance = distance * 1000
            total_distance += distance
            duration = distance/4500 #speed of walking 4.5 km/hr
            total_duration += duration
            weight = 1
            total_weight += 1
        
        #add steps
        if n > 2 : # a point after turn
            w = aroute[n-2]
            b1 = calculate_initial_compass_bearing(w, z)
            b2 = calculate_initial_compass_bearing(z, each)
            turn = abs(b1-b2);
            if (turn > 180):
                turn = 360 - turn
            
            #turn = abs(calcDir(w,z,each)[1])

            #if not 160 <= turn <= 200:
            if   turn <= 160 :
                modifier = calcDir(w,z,each)[0]
                turn_points.append(z)
                n_turn += 1
                # add turn
                data["legs"][0]["steps"].append({"distance":0, # tobe updated when a turn occur
                                 "intersections":[{"location":[z[0],z[1]],"bearings":[calculate_initial_compass_bearing(z, each)], "entry":[True], "out": 0, "in":0}], # leave blanks for now
                                                  "driving_side":"left", # always left
                                                  "geometry":0,
                                                  "mode":"walking",
                                                  "maneuver":{},
                                                  "ref":0, 
                                                  "weight":0, # tobe updated when a turn occur
                                                  "duration":0, # tobe updated when a turn occur
                                                  "name":"street name",
                                                  "distance":0}) # tobe updated when a turn occur

                data["legs"][0]["steps"][n_turn]["maneuver"] = {"bearing_after":calculate_initial_compass_bearing(z, each), # tobe updated when a turn occur
                                                                "bearing_before":calculate_initial_compass_bearing(w, z), 
                                                                "location":[z[0],z[1]],
                                                                "modifier":modifier,
                                                                "type":"turn",
                                                                "instruction":"Turn " + modifier }
                
                data["legs"][0]["steps"][n_turn]["voiceInstructions"] = [{"distanceAlongGeometry":0,# tobe updated when a turn occur
                                                          "announcement":"Turn " + modifier,
                                                          "ssmlAnnouncement":"\u003cspeak\u003e\u003camazon:effect name\u003d\"drc\"\u003e\u003cprosody rate\u003d\"1.08\"\u003e Head on\u003c/prosody\u003e\u003c/amazon:effect\u003e\u003c/speak\u003e"}]
                data["legs"][0]["steps"][n_turn]["bannerInstructions"] = [{
                    "distanceAlongGeometry":0, # tobe updated when a turn occur
                    "primary":{
                        "text":"Street",
                        "components":[{
                            "text":"Street",
                            "type":"text",
                            "abbr":True,
                            "abbr_priority":0}
                        ],
                        "type":"turn", # tobe updated when a turn occur
                        "modifier":modifier}}] # tobe updated when a turn occur
                
                ## update previous turn
                prev_weight = total_weight - weight - prev_weight
                prev_distance = total_distance - distance - prev_distance
                prev_duration = total_duration - duration - prev_duration
                data["legs"][0]["steps"][n_turn-1]["weight"] = round(prev_weight,1) 
                data["legs"][0]["steps"][n_turn-1]["duration"] = round(prev_duration,1)
                data["legs"][0]["steps"][n_turn-1]["distance"] = round(prev_distance,1)
                data["legs"][0]["steps"][n_turn-1]["geometry"] = polyline.encode(swift(leg_points[:-1]),6)
                data["legs"][0]["steps"][n_turn-1]["voiceInstructions"][0]["distanceAlongGeometry"] = round(prev_distance,1)
                data["legs"][0]["steps"][n_turn-1]["voiceInstructions"].append({"distanceAlongGeometry": 50,# tobe updated when a turn occur
                                                          "announcement":"Turn " + modifier,
                                                          "ssmlAnnouncement":"\u003cspeak\u003e\u003camazon:effect name\u003d\"drc\"\u003e\u003cprosody rate\u003d\"1.08\"\u003e In 50 meters Turn " + modifier + "\u003c/prosody\u003e\u003c/amazon:effect\u003e\u003c/speak\u003e"
                                                          })
                
                data["legs"][0]["steps"][n_turn-1]["bannerInstructions"][0]["distanceAlongGeometry"] = round(prev_distance,1)
                data["legs"][0]["steps"][n_turn-1]["bannerInstructions"].append({
                    "distanceAlongGeometry":10, # tobe updated when a turn occur
                    "primary":{
                        "text":"Street",
                        "components":[{
                            "text":"Street",
                            "type":"text",
                            "abbr":True,
                            "abbr_priority":0}
                        ],
                        "type":"turn", # tobe updated when a turn occur
                        "modifier":modifier }})
                leg_points = [each]

        z = each
        n += 1
    #add arrive
    data["legs"][0]["steps"].append({"distance":0, # tobe updated when a turn occur
                                 "intersections":[{"location":[aroute[-1][0], aroute[-1][1]],"bearings":[calculate_initial_compass_bearing(aroute[-2], aroute[-1])], "entry":[True], "out": 0, "in":0}], # leave blanks for now
                                                      "driving_side":"left", # always left
                                                      "geometry":polyline.encode(swift([aroute[-1]]),6),
                                                      "mode":"walking",
                                                      "maneuver":{"bearing_after":0, # tobe updated when a turn occur
                                                                  "bearing_before":calculate_initial_compass_bearing(aroute[-2], aroute[-1]), # tobe updated when a turn occur
                                                                  "location":[aroute[-1][0], aroute[-1][1]],
                                                                  "type":"arrive",
                                                                  "instruction":"You have arrived"},
                                                      "ref":0, 
                                                      "weight":0, # tobe updated when a turn occur
                                                      "duration":0, # tobe updated when a turn occur
                                                      "name":"street name",
                                                      "distance":0}) # tobe updated when a turn occur
    data["legs"][0]["steps"][-1]["voiceInstructions"] = []
    data["legs"][0]["steps"][-1]["bannerInstructions"] = [] 
    
    prev_weight = total_weight - weight - prev_weight
    prev_distance = total_distance - distance - prev_distance
    prev_duration = total_duration - duration - prev_duration
    data["legs"][0]["steps"][n_turn]["weight"] = round(total_weight - prev_weight,1)
    data["legs"][0]["steps"][n_turn]["duration"] = round(total_distance - prev_distance,1)
    data["legs"][0]["steps"][n_turn]["distance"] = round(total_duration - prev_duration,1)
    data["legs"][0]["steps"][n_turn]["geometry"] = polyline.encode(swift([z,aroute[-1]]),6)
    data["legs"][0]["steps"][n_turn]["voiceInstructions"][0]["distanceAlongGeometry"] = round(total_distance - prev_distance,1)
    data["legs"][0]["steps"][n_turn]["voiceInstructions"].append({"distanceAlongGeometry":50,# tobe updated when a turn occur
                                                          "announcement":"In 10 meter You will arrive",
                                                          "ssmlAnnouncement":"\u003cspeak\u003e\u003camazon:effect name\u003d\"drc\"\u003e\u003cprosody rate\u003d\"1.08\"\u003eIn 50 meters You will arrive\u003c/prosody\u003e\u003c/amazon:effect\u003e\u003c/speak\u003e"
                                                          })
    
    data["legs"][0]["steps"][n_turn]["bannerInstructions"][0]["distanceAlongGeometry"] = round(total_distance - prev_distance,1)
    data["legs"][0]["steps"][n_turn]["bannerInstructions"].append({
                    "distanceAlongGeometry":10, # tobe updated when a turn occur
                    "primary":{
                        "text":"You will arrive",
                        "components":[{
                            "text":"You will arrive",
                            "type":"text",
                            "abbr":True,
                            "abbr_priority":0}
                        ],
                        "type":"arrive"}}) # tobe updated when a turn occur
    
    # edit summary
    turn_points.append(aroute[-1])
    data["weight"] = round(total_weight,1)
    data["distance"] = round(total_distance,1)
    data["duration"] = round(total_duration,1)
    data["geometry"] = polyline.encode(swift(waypoints),6)
    #data["geometry"] = "r{kdgA}ewosGd@{GXyDxEwq@jFiz@f@eI\\{FjJqzAd@wGXiEzC}c@~F{{@`HcdADq@r@iNf@eN`@eNn@kMjDak@`@wG[EoEe@uFo@oEk@ct@mJiWeDqa@_FoNqBoiA{No]iE"
    data["legs"][0]["weight"] = round(total_weight,1)
    data["legs"][0]["distance"] = round(total_distance,1)
    data["legs"][0]["duration"] = round(total_duration,1)
    return data



# In[6]:



pickle_in = open("../pickle/G.pickle","rb")
G = pickle.load(pickle_in)
pickle_in.close()

pickle_in = open("../pickle/N.pickle","rb")
nodes = pickle.load(pickle_in)
pickle_in.close()

pickle_in = open("../pickle/D.pickle","rb")
dTree = pickle.load(pickle_in)
pickle_in.close()





app = Flask(__name__)
api = Api(app)



    
class SDK(Resource):
    def get(self, source, destination):
      source = [(float(source.split(",")[0]),float(source.split(",")[1]))]
      destination = [(float(destination.split(",")[0]), float(destination.split(",")[1]))]
      #result lat long
      sourceMap = nodes[dTree.query([source[0][1], source[0][0]])[1]]
      #result lat long
      destinationMap = nodes[dTree.query([destination[0][1], destination[0][0]])[1]]

      routes = nx.shortest_path(G,(sourceMap[1], sourceMap[0]),(destinationMap[1], destinationMap[0]), weight = "weight")
      routes = source + routes + destination
      
      return jsonify(transformerSDK(routes))




api.add_resource(SDK, '/sdk/<source>/<destination>')



if __name__ == '__main__':
    app.debug = True
    app.run(host = '0.0.0.0',port='5502')



# In[9]:




