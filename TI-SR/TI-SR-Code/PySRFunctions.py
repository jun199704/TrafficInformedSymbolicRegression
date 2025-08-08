import json
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import copy


import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.special import erf

import logging
from numpy import linalg as LA
   
import json
from sklearn.model_selection import train_test_split
import time


import multiprocessing
import socket
import struct
import concurrent.futures

import pandas as pd
import os

'''

def generate_excel(vehTrajsAtk,item):
    
    vehIDList=[]
    for vehID in vehTrajsAtk.keys():
        vehIDList.append(vehID)

    singleDict={}
    for vehID in vehTrajsAtk.keys():
        singleDict[vehID]=copy.deepcopy(vehTrajsAtk[vehID][item])
    
    lenList=[]
    for vehID in singleDict.keys():
        lenList.append(len(singleDict[vehID]))
    maxLen=max(lenList)
    
    for vehID in singleDict.keys():
        idx=maxLen-len(singleDict[vehID])
        for i in range(idx):
            singleDict[vehID].append(10000)

    datasetID=vehTrajsAtk[vehID]['DatasetID']
            
    # Convert the dictionary to a pandas DataFrame        
    df = pd.DataFrame(singleDict)   
    
    # Write the DataFrame to an Excel file
   
    filename='/home/drivesim/Documents/SR/MSFTrainData/'+item+datasetID+'PySR.csv'
    #print(filename)

    if os.path.exists(filename):
        os.remove(filename)

    df.to_csv(filename, index=False)  # index=False prevents adding a row index to the file
    
    return vehIDList

'''

import os
import copy
import csv

import os
import copy
import csv

def generate_excel(vehTrajsAtk, item):
    vehIDList = list(vehTrajsAtk.keys())

    singleDict = {
        vehID: copy.deepcopy(vehTrajsAtk[vehID][item])
        for vehID in vehTrajsAtk.keys()
    }

    maxLen = max(len(singleDict[vehID]) for vehID in singleDict.keys())

    for vehID in singleDict.keys():
        idx = maxLen - len(singleDict[vehID])
        singleDict[vehID].extend([10000] * idx)

    datasetID = vehTrajsAtk[vehIDList[0]]['DatasetID']

    # Prepare file path
    filename = f'/home/drivesim/Documents/SR/MSFTrainData/{item}{datasetID}PySR.csv'

    if os.path.exists(filename):
        os.remove(filename)

    # Write the data to the CSV file
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Write headers (vehicle IDs)
        writer.writerow(vehIDList)

        # Write rows
        for row in zip(*(singleDict[vehID] for vehID in vehIDList)):
            writer.writerow(row)

    return vehIDList



def calculate_response(vehTrajsAtk, server_socket,receive_socket,send_port,receive_port):

    dataList=[1]
            
    server_address = ('localhost', send_port)
    json_data = json.dumps(dataList)
    start_time = time.time()
    server_socket.sendto(json_data.encode('utf-8'), server_address)
    #print("finish sending")
    # Receive data
    data_received, addr = receive_socket.recvfrom(4096)  # Buffer size of 4096 bytes
    end_time = time.time()
    runtime = end_time - start_time
    #print(f"Runtime: {runtime} seconds")
    #print("data_received")
    # Decode the received byte array to a JSON string
    json_str_received = data_received.decode('utf-8')

    # Convert JSON string to a Python dictionary
    received_variable = json.loads(json_str_received)
    #print(received_variable)

    return received_variable

def generate_ID_file(vehIDList, datasetID):
    
    
    # Create a DataFrame from the complexity list
    df = pd.DataFrame(vehIDList, columns=["vehID"])


    filename = '/home/drivesim/Documents/SR/MSFTrainData/'+'vehID'+datasetID+'PySRTrain.xlsx'

    df.to_excel(filename, index=False)
    # Deep copy the list into the dictionary (if needed for further use)


def process_trajectory(datasetID, singlevehTrajs, send_port, receive_port, server_socket,receive_socket):
    result={}
    
    #print("start processing trajectory")
    
                       
    error_single= calculate_response(singlevehTrajs,server_socket, receive_socket, send_port, receive_port)                    

    result[datasetID]=error_single 
    # Close the sockets
    server_socket.close()
    receive_socket.close()
    
    return result

#datasetIDList= ['06','36_1','36_2','37']
#for datasetID in ['06','36_1','36_2','37']:
#    filename = "/home/drivesim/Documents/SR/MSFTrainData/TrainingDataset"+str(datasetID)+".json"
#    with open(filename, 'r') as f:
#        vehTrajsAtk = json.load(f)

def parallel_calculate_CR_matrix(traj_dict):

    latdev_threshold=1.2

    avgCRList=[]
    # Load your data here
    vehTrajsAtkAll={}
    
    #print("trajectory dict length")
    #print(len(traj_dict))
    #print(traj_dict.keys())
    
    datasetIDList= ['36_1','36_2','37']#,'36_2','37']
    for datasetID in ['36_1','36_2','37']:#,'36_2','37']:
        filename = "/home/drivesim/Documents/SR/MSFTrainData/TrainingDataset"+str(datasetID)+".json"
        with open(filename, 'r') as f:
            vehTrajsAtk = json.load(f)
   
        delVehList=[]
        for vehID in vehTrajsAtk.keys():
            if vehID not in traj_dict.keys():
                delVehList.append(vehID)

        for vehID in delVehList:
            del vehTrajsAtk[vehID]
        #print(vehTrajsAtk.keys())
        #print(delVehList)

        #print("vehicle length")
        #print(len(vehTrajsAtk))
        #print("training dataset size: "+str(len(vehTrajsAtk)))


        for vehID in vehTrajsAtk.keys():
            traj_dict[vehID]['latdev']=list(traj_dict[vehID]['latdev'])
            vehTrajsAtk[vehID]['latdevMSF']=copy.deepcopy(vehTrajsAtk[vehID]['latdev'])
            vehTrajsAtk[vehID]['latdev']=traj_dict[vehID]['latdev'] 

        for vehID in vehTrajsAtk.keys():
            vehTrajsAtkAll[vehID]=copy.deepcopy(vehTrajsAtk[vehID])
    
    #datasetIDList=[['06','36_1'],['36_2','37']]
    for datasetID in datasetIDList:
        #print(datasetID)
        delVehList=[]
        for vehID in vehTrajsAtkAll.keys():
            if vehTrajsAtkAll[vehID]['DatasetID'] not in datasetID:
                    delVehList.append(vehID)

        singlevehTrajs=copy.deepcopy(vehTrajsAtkAll)
        for vehID in delVehList:
            del singlevehTrajs[vehID]
                    
        for vehID in singlevehTrajs.keys():
            singlevehTrajs[vehID]['distanceAtk']=copy.deepcopy(singlevehTrajs[vehID]['distanceGT'])
        
        #print(len(singlevehTrajs))
        vehIDList_speed=generate_excel(singlevehTrajs,'speed')
        vehIDList_distanceAtk=generate_excel(singlevehTrajs,'distanceAtk')
        vehIDList_accel=generate_excel(singlevehTrajs,'accel')
        vehIDList_latdev=generate_excel(singlevehTrajs,'latdev')        

    # calculate lane departure time
    '''
    lanedepartTimeDiffList=[]
    for vehID in vehTrajsAtkAll.keys():
        lanedepartTime=0
        for i in range(len(vehTrajsAtkAll[vehID]['latdev'])):
            if abs(vehTrajsAtkAll[vehID]['latdev'][i])>latdev_threshold:
                lanedepartTime=i/10
                break
        lanedepartTimeDiff=abs(lanedepartTime-vehTrajsAtkAll[vehID]['laneDepartTime'])
        lanedepartTimeDiffList.append(lanedepartTimeDiff)
    #print(lanedepartTimeDiffList)
    '''

    
    port_base = 50030  # Starting port number
    results = {}
    
    #datasetIDList=[['06','36_1'],['36_2','37']]
    with concurrent.futures.ProcessPoolExecutor() as executor:
                futures = []
                
                # Iterate over followSpeedDiffLists in parallel
                #for i, followSpeedDiff in enumerate(followSpeedDiffList):
                for i, datasetID in enumerate(datasetIDList):

                    delVehList=[]
                    for vehID in vehTrajsAtkAll.keys():
                        if vehTrajsAtkAll[vehID]['DatasetID'] not in datasetID:
                            delVehList.append(vehID)

                    singlevehTrajs=copy.deepcopy(vehTrajsAtkAll)
                    for vehID in delVehList:
                        del singlevehTrajs[vehID]
                    
                    for vehID in singlevehTrajs.keys():
                        singlevehTrajs[vehID]['distanceAtk']=copy.deepcopy(singlevehTrajs[vehID]['distanceGT'])

                    #vehIDList_speed=generate_excel(singlevehTrajs,'speed',i)
                    #vehIDList_distanceAtk=generate_excel(singlevehTrajs,'distanceAtk',i)
                    #vehIDList_accel=generate_excel(singlevehTrajs,'accel',i)
                    #vehIDList_latdev=generate_excel(singlevehTrajs,'latdev',i)
                    
                    send_port = port_base + 2*i
                    receive_port=port_base + 2*i-1
                    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    #server_socket.bind(('localhost', send_port))  # Bind to the provided port
                    receive_socket.bind(('localhost',receive_port))

                    
                    #filename='ID2IDX.json'
                    #with open(filename, 'r') as f:
                    #    vehIDDict = json.load(f)

                    #vehIdxList=[]
                    #for vehID in vehIDList_speed:
                    #    vehIdxList.append(vehIDDict[vehID])

                    #generate_ID_file(vehIdxList, datasetID)

                    #print(i)
                    futures.append(executor.submit(
                        process_trajectory, i, singlevehTrajs, send_port, receive_port, server_socket,receive_socket))
        
                for future in concurrent.futures.as_completed(futures):
                    result = future.result()
                    results.update(result)

    #print(results)
    resultList=[]
    for itemID in results.keys():
        resultList.append(results[itemID])

    #print(resultList)
    mean_error=np.mean(resultList)  
    
    #mean_lane_depart_time=np.mean(lanedepartTimeDiffList) 
        
 
    # Create argument list for each iteration
    '''   
    port_base = 50030  # Starting port number
  

    send_port = port_base 
    receive_port = port_base - 1

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #server_socket.bind(('localhost', send_port))  # Bind to the provided port
    receive_socket.bind(('localhost',receive_port))

    mean_error = calculate_response(vehTrajsAtkAll, server_socket, receive_socket, send_port, receive_port)  
    '''
    return mean_error# mean_error, mean_lane_depart_time

def get_best_candidates(best_candidate):
    # Get a list of keys sorted by their corresponding values in ascending order
    #print(best_candidate)
    sorted_candidates = [k for k, v in sorted(best_candidate.items(), key=lambda item: item[1])]

    # Print the sorted keys
    #print(sorted_candidates)
    return sorted_candidates

    

