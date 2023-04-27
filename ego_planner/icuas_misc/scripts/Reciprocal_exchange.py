# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 13:47:58 2023

@author: Canning
"""

import numpy as np
import random
def Reciprocal_exhange(child, no_cities):
    child = np.reshape(child, (1,no_cities))
    while True:
        index = [random.randint(0,no_cities-1), random.randint(0,no_cities-1)]
        if index[0] != index[1]:
            break
    
    content = [child[0,index[0]], child[0,index[1]]]
    new_child = child
    new_child[0,index[0]] = content[1]
    new_child[0,index[1]] = content[0]
    return new_child