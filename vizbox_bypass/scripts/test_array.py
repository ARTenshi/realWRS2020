#!/usr/bin/emv python
#! -*- coding:utf-8 -*-

import numpy as np


outcome_from = ['MOVE2SEARCH','MOVE2SEARCH', 'MOVE2SEARCH', 'MOVE2SEARCH', 'ALIGN2FURNITURE', 'ALIGN2FURNITURE', 'ALIGN2FURNITURE', 'ALIGN2FURNITURE', 'DETECTGRASPOBJECTONFLOOR', 'DETECTGRASPOBJECTONFLOOR','DETECTGRASPOBJECTONFLOOR', 'DETECTGRASPOBJECTONFLOOR', 'DECISIONSYSTEM','DECISIONSYSTEM', 'TAKEOBJECTONFURNITURE', 'TAKEOBJECTONFURNITURE','MOVE2TIMEOUT', 'MOVE2TIMEOUT', 'STARTTRIAL', 'STARTTRIAL','GRASPVISUALONFURNITURE', 'GRASPVISUALONFURNITURE','GRASPVISUALONFURNITURE', 'ALIGN2SHELF', 'ALIGN2SHELF', 'ALIGN2SHELF','ALIGN2SHELF', 'ASK_LOCATION', 'ASK_LOCATION', 'ASK_LOCATION','ASK_LOCATION', 'START', 'START', 'MOVE2SHELF', 'MOVE2SHELF', 'DETECTSPACE','DETECTSPACE', 'DETECTSPACE', 'DETECTSPACE', 'MOVE2STANDBY', 'MOVE2STANDBY','DETECTOBJECTSONFURNITURE', 'DETECTOBJECTSONFURNITURE','DETECTOBJECTSONFURNITURE', 'DETECTOBJECTSONFURNITURE','CONFIRM_LOCATION', 'CONFIRM_LOCATION', 'CONFIRM_LOCATION', 'WAIT_DOOR', 'WAIT_DOOR', 'WAIT_DOOR', 'WAIT_HAND', 'WAIT_HAND', 'WAIT_HAND','CHECKGRASPING', 'CHECKGRASPING', 'CHECKGRASPING', 'CHECKGRASPING','CHECKGRASPING2', 'CHECKGRASPING2', 'CHECKGRASPING2']

outcome_to =  ['failure', 'ALIGN2FURNITURE', 'success', 'DETECTGRASPOBJECTONFLOOR','DETECTOBJECTSONFURNITURE', 'MOVE2TIMEOUT', 'DETECTOBJECTSONFURNITURE','failure', 'STARTTRIAL', 'MOVE2TIMEOUT', 'CHECKGRASPING', 'failure','failure', 'MOVE2SEARCH', 'failure', 'CHECKGRASPING2', 'failure', 'success','failure', 'MOVE2SEARCH', 'failure', 'MOVE2TIMEOUT','TAKEOBJECTONFURNITURE', 'DETECTSPACE', 'MOVE2TIMEOUT', 'DETECTSPACE','failure', 'failure', 'ASK_LOCATION', 'CONFIRM_LOCATION','CONFIRM_LOCATION', 'failure', 'STARTTRIAL', 'failure', 'ALIGN2SHELF','failure', 'MOVE2TIMEOUT', 'MOVE2SEARCH', 'failure', 'failure', 'success','STARTTRIAL', 'MOVE2TIMEOUT', 'GRASPVISUALONFURNITURE', 'failure','ASK_LOCATION', 'failure','DECISIONSYSTEM', 'None', 'failure', 'START','failure', 'WAIT_DOOR', 'WAIT_DOOR', 'STARTTRIAL', 'failure', 'MOVE2SEARCH','MOVE2SHELF', 'failure', 'MOVE2SEARCH', 'MOVE2SHELF']



result_array = []
search_array = []
already_search_array = []
initial_state = ['WAIT_HAND']
container_outcomes = ['success', 'failure']
result_array.append(initial_state[0])

array = np.array((outcome_from, outcome_to))

while not np.all(array == 'None'): # ここの条件をどうするか
    #search_array = (list(set(result_array) ^ set(search_array)))
    search_array = (list(set(result_array) ^ set(already_search_array)))
    y = np.array([])
    for j in search_array:
        y = np.append(y, np.where(array[0] == j))
    if  len(y) == 0: break
    print 'result array is ' + str(result_array)
    print 'search_array is'  + str(search_array)
    print 'array is ' + str(array)
    print 'y' + str(y)
    for wai in y:
        now_search_state = array[0+1, int(wai)]
        print 'now_search_state is ' + str(now_search_state)
        
        # 最終outcomeは追加しない     
        if  any(i == now_search_state for i in container_outcomes):
            print 'not append becaouse it is final outcome'
            array[0+1, int(wai)] = None
            array[0, int(wai)] = None
            continue
        
        # すでに同じステートが追加されてたら追加しない
        elif  any(k == now_search_state for k in result_array):
            print 'not append becaouse it is already in result state'
            array[0+1, int(wai)] = None
            array[0, int(wai)] = None
            continue
        # searchにNoneがある可能性もあるからそれはパス
        elif now_search_state == 'None':
            print 'not append because it is None'
            continue
        #yが空だったらreturn
        else:
            print 'append'
            result_array.append(str(array[0+1, int(wai)]))
            for h in search_array:
                already_search_array.append(h)
            array[0+1, int(wai)] = None
            array[0, int(wai)] = None
                
    print '------------------------------'
                    
print result_array
print 'endddddddddddddddddddddd'
