#!/usr/bin/env python
# license removed for brevity
import os
import sys

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import random
from crossover import crossover
from Reversion_mutation import Reversion_mutation
from Reciprocal_exchange import Reciprocal_exhange
from numpy import linalg as LA
import time

from icuas23_competition.msg import poi


def TSP(poi_array, starting_pt):
    #Initialize cities

    point_array = np.empty((0,3), float)

    

    for i in range(len(poi_array)):
        print(i)
        x = poi_array[i].x
        y = poi_array[i].y
        z = poi_array[i].z

        point_array = np.vstack((point_array, [x, y, z]))


        # city = np.array([poi_array[i].x, poi_array[i].y, poi_array[i].z])
        # all_cities = np.append(city,  np.reshape(city, (1,3)), axis = 0)
        # print("here in TSP!")
        # print(i)

    origin_x = starting_pt.pose.position.x
    origin_y = starting_pt.pose.position.y
    origin_z = starting_pt.pose.position.z

    point_array = np.vstack((point_array, [origin_x, origin_y, origin_z]))


    tick = time.time()
    
 

    #Assign depot
    # depot = np.array([origin_x, origin_y, origin_z])
    Cities = point_array
    no_cities = len(Cities)

    print("HERE ARE THE CITIES!!!!")
    print(no_cities)
    print(Cities)

    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    

    #Initilize GA Parameters
    no_gen = 300 #No of iterations
    no_population = 50 #No of individual in each iteration
    parent_pool = 10
    elitism_rate = 0.1 #No of individual to preserve in the next gen
    crossover_rate = 0.9
    mutation_rate = 0.1

    #Initilize first generation
    current_gen = np.zeros((no_population, no_cities),dtype=np.int32)
    for i in range(no_population):
        current_gen[i,:] = np.random.permutation(no_cities)
    cg_fitness = np.zeros((no_population, 1))
    for i in range(no_population):
        distance = 0
        for ii in range(no_cities-1):
            distance = distance + LA.norm(Cities[current_gen[i,ii+1],:]-Cities[current_gen[i,ii],:])
        distance = distance + LA.norm(Cities[current_gen[i,-1],:]-Cities[current_gen[i,0],:])
        cg_fitness[i] = 1/distance

    id = np.argsort(-cg_fitness, axis=0)
    cg_fitness = cg_fitness[id]
    cg_fitness = cg_fitness.reshape(no_population,1)
    current_gen = current_gen[id,:]
    current_gen = current_gen.reshape(no_population, no_cities)

    for i in range(1,no_gen):
        new_gen = np.zeros((no_population, no_cities),dtype=np.int32)
        ng_fitness = np.zeros((no_population, 1))
        new_gen[0:np.int_(np.rint(no_population*elitism_rate)),:] = current_gen[0:np.int_(np.rint(no_population*elitism_rate)),:]
        ng_fitness[0:np.int_(np.rint(no_population*elitism_rate)),:] = cg_fitness[0:np.int_(np.rint(no_population*elitism_rate)),:]
        fitness_sum = sum(cg_fitness)
        prob = np.reshape(cg_fitness/fitness_sum, (-1))
        for ii in range(np.int_(np.rint(no_population*elitism_rate)), no_population):
            index = np.random.choice(no_population,parent_pool, p=prob)
            parent_fitness = cg_fitness[index]
            parent_id = np.argsort(-parent_fitness, axis=0)
            index = index[parent_id]
            parent1 = current_gen[index[0],:]
            loop_id = 0
            while True:
                if index[0] != index[loop_id+1]:
                    parent2 = current_gen[index[loop_id+1],:]
                    break
                loop_id = loop_id + 1
            
            if random.uniform(0, 1) <= crossover_rate:
                child = crossover(parent1, parent2, no_cities, Cities)
            else:
                child = parent1
                
            if random.uniform(0,1) <= mutation_rate:
                child = Reversion_mutation(child, no_cities)
                
            if random.uniform(0,1) <= mutation_rate:
                child = Reciprocal_exhange(child, no_cities)
                
            new_gen[ii,:] = child
            distance = 0
            for iii in range(no_cities-1):
                distance = distance + LA.norm(Cities[new_gen[ii,iii+1],:]-Cities[new_gen[ii,iii],:])
            
            distance = distance + LA.norm(Cities[new_gen[ii,-1],:]-Cities[new_gen[ii,0],:])
            ng_fitness[ii] = 1/distance
        id = np.argsort(-ng_fitness, axis=0)
        cg_fitness = ng_fitness[id]
        cg_fitness = cg_fitness.reshape(no_population,1)
        current_gen = new_gen[id,:]
        current_gen = current_gen.reshape(no_population, no_cities)
        
    # print(current_gen)
    best_solution = current_gen[0,:]
    best_solution = best_solution.reshape(1, no_cities)
    print("\n")
    print("\n")
    print(best_solution)


    where_is_depot = 0
    for i in range(no_cities):
        if best_solution[0][i] == no_cities - 1:
            where_is_depot = i

    best_solution_right_order = []



    # now do reordering
    for i in range(where_is_depot + 1, no_cities):
        # print(best_solution[0][i])
        best_solution_right_order.append(best_solution[0][i])

    print("\n")
    # print("\n")
    for i in range(where_is_depot + 1):
        # print(best_solution[0][i])
        best_solution_right_order.append(best_solution[0][i])

    print("gan")     

    print(best_solution_right_order)   
       

    final_poi_array = Path()

    for i in range(no_cities):
        # print(best_solution[0][i])
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = point_array[best_solution_right_order[i]][0]
        pose.pose.position.y = point_array[best_solution_right_order[i]][1]
        pose.pose.position.z = point_array[best_solution_right_order[i]][2]

        final_poi_array.poses.append(pose)

        # print(point_array[best_solution_right_order[i]][0])
        # print(point_array[best_solution_right_order[i]][1])
        # print(point_array[best_solution_right_order[i]][2])
        print("--------")
        # final_poi_array = np.vstack((final_poi_array, point_array[best_solution_right_order[i]]))
    
    # print(final_poi_array)


    best_distance = 1/cg_fitness[0]
    best_distance = best_distance + LA.norm(Cities[best_solution[0, -1],:]-Cities[best_solution[0, 0],:])
    tock = time.time()
    print(best_distance)
    print(tock-tick)

    return final_poi_array


class process_POI:
    def __init__(self):
        
        # Create a pub and sub
        self.pose_sub = rospy.Subscriber('/red/pose', PoseStamped, callback=self.pose_callback)
        self.poi_sub = rospy.Subscriber('/red/poi', poi, callback=self.poi_callback)
        self.poi_tsp_pub = rospy.Publisher('/tsped_pois', Path, queue_size=1)
        self.poi_tsp_cal_timer = rospy.Timer(rospy.Duration(0.02), self.calculate_tsp_POI)
        self.poi_tsp_pub_timer = rospy.Timer(rospy.Duration(0.02), self.publish_tsped_POI)
        
        # class object
        self.got_pose = False
        self.pose = PoseStamped()

        self.got_POI = False
        self.can_pub_POI = False
        
        self.tsped_pois = Path()
        self.poiarray = poi()

    def pose_callback(self, data):
        self.got_pose = True
        self.pose = data

    def poi_callback(self, data):

        print("Here POI Callback!")

        self.got_POI = True
        self.poiarray = data

        if(self.can_pub_POI == False):
            print(self.poiarray.poi[0])

    def calculate_tsp_POI(self, event=None):
        # print("Here Cal TSP")
        # TSP()
        if(self.got_pose == True and self.got_POI == True and self.can_pub_POI == False):
            print("Here Cal TSP")                        

            self.tsped_pois = TSP(self.poiarray.poi, self.pose)

            self.can_pub_POI = True


    def publish_tsped_POI(self, event=None):
        if(self.can_pub_POI == True):
            self.poi_tsp_pub.publish(self.tsped_pois)  


if __name__ == '__main__':
    
    rospy.init_node("TSP")
    # Create an instance of Temperature sensor
    ts = process_POI()
    print("hi")
    
    rospy.spin()