# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 12:55:17 2023

@author: Canning
"""
"""
For the matlab code, the "best" method that will yield the optimal solution utilize a 
math solver, which is difficult to translate. So this solver uses a search algo (GA)
"""
#%matplotlib widget

import numpy as np
import random
from crossover import crossover
from Reversion_mutation import Reversion_mutation
from Reciprocal_exchange import Reciprocal_exhange
from numpy import linalg as LA
import time
# import matplotlib.pyplot as plt

#Initialize cities
tick = time.time()
x_bound = 25
y_bound = 50
z_bound = 15
no_cities = 10
x = np.random.randint(1,x_bound,size = (no_cities,1))
y = np.random.randint(1,y_bound,size = (no_cities,1))
z = np.random.randint(1,z_bound,size = (no_cities,1))
Cities = np.hstack((x,y,z))


#Assign depot
depot = np.array([0.2,0,0])
Cities = np.append(Cities,  np.reshape(depot, (1,3)), axis = 0)
print(Cities)
no_cities = no_cities+1

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
print(best_solution)
best_distance = 1/cg_fitness[0]
best_distance = best_distance + LA.norm(Cities[best_solution[0, -1],:]-Cities[best_solution[0, 0],:])
tock = time.time()
print(best_distance)
print(tock-tick)
