# -*- coding: utf-8 -*-
"""
Created on Wed Jan 25 16:28:05 2023

@author: Asus
"""

        
import os
import datetime
def GetFileName():
        x = datetime.datetime.now()
        s = x.strftime('%Y-%m-%d-%H%M%S%f')
        return s
def CreateDir(path):
    ls = [];
    head_tail = os.path.split(path)
    ls.append(path)
    while len(head_tail[1])>0:
        head_tail = os.path.split(path)
        path = head_tail[0]
        ls.append(path)
        head_tail = os.path.split(path)   
    for i in range(len(ls)-2,-1,-1):
        sf =ls[i]
        isExist = os.path.exists(sf)
        if not isExist:
            os.makedirs(sf)
            
            
            
sf = GetFileName()
sdir = "d:\\a\\b\\c"
CreateDir(sdir)
    
    
    
    