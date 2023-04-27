# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 18:08:02 2023

@author: Canning
"""

import numpy as np
import random
from numpy import linalg as LA
def crossover(parent1, parent2, num_point, Cities):
    child = np.zeros((1, num_point), dtype=np.int32)-1
    current = random.randint(0, num_point-1)
    child[0,0] = current
    for i in range(1,num_point):
        repeat_flag1 = 0
        repeat_flag2 = 0
        temp_id = np.where(parent1 == current)
        temp_id = temp_id[1]
        if temp_id == num_point-1:
            temp_id = 0
        else:
            temp_id = temp_id + 1
        
        parent1_id = current
        parent1_id2 = parent1[0,temp_id]
        temp_id = np.where(parent2 == current)
        temp_id = temp_id[1]
        if temp_id == num_point-1:
            temp_id = 0
        else:
            temp_id = temp_id + 1
        parent2_id = current
        parent2_id2 = parent2[0,temp_id]
        parent1_dist = LA.norm(Cities[parent1_id2,:]-Cities[parent1_id,:])
        parent2_dist = LA.norm(Cities[parent2_id2,:]-Cities[parent2_id,:])
        if parent1_id2 in child:
            repeat_flag1 = 1
        if parent2_id2 in child:
            repeat_flag2 = 1
        if (parent1_dist < parent2_dist and repeat_flag1 == 0) or (repeat_flag1 == 0 and repeat_flag2 == 1):
            child[0,i] = parent1_id2
            current = parent1_id2
            continue
        if (parent2_dist < parent1_dist and repeat_flag2 == 0) or (repeat_flag1 == 1 and repeat_flag2 == 0):
            child[0,i] = parent2_id2
            current = parent2_id2
            continue
        if (parent1_dist == parent2_dist) or (repeat_flag1 == 1 and repeat_flag2 == 1):
            if repeat_flag1 == 0 and repeat_flag2 == 1:
                child[0,i] = parent1_id2
                current = parent1_id2
                continue
            elif repeat_flag2 == 0 and repeat_flag1 == 1:
                child[0,i] = parent2_id2
                current = parent2_id2
                continue
            elif repeat_flag1 == 0 and repeat_flag2 == 0:
                child[0,i] = parent1_id2
                current = parent1_id2
                continue
            elif repeat_flag1 == 1 and repeat_flag2 == 1:
                while True:
                    repeat_flag3 = 0
                    current = random.randint(0, num_point-1)
                    if current in child:
                        repeat_flag3 = 1
                    if repeat_flag3 == 0:
                        child[0,i] = current
                        break
    return child