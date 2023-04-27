# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 12:00:50 2023

@author: Canning
"""

import numpy as np
import random
def Reversion_mutation(child, no_cities):
    while True:
        index = [random.randint(1, no_cities-1), random.randint(1, no_cities-1)]
        if (index[0] != index[1]) and (index[0]+1 < index[1]):
            break
    content1 = child[0,0:index[0]]
    content2 = child[0,index[0]:index[1]]
    content2 = np.flip(content2,0)
    if child[0,index[1]:-1].size == 0:
        content3 = np.array([child[0,-1]])
    else:
        content3 = child[0,index[1]:]
    child = np.concatenate((content1, content2, content3))
    
    return child
            